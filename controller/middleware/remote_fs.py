from pathlib import Path
from typing import Any

from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.observability.middleware_helper import (
    broadcast_file_update,
    record_events,
    record_simulation_result,
    sync_file_asset,
)
from controller.workflows.execution import ScriptExecutionWorkflow
from controller.workflows.heavy import (
    HeavyPreviewWorkflow,
    HeavySimulationWorkflow,
    HeavyValidationWorkflow,
)
from shared.observability.schemas import (
    EditFileToolEvent,
    GrepToolEvent,
    LibraryUsageEvent,
    LsFilesToolEvent,
    ManufacturabilityCheckEvent,
    PlanSubmissionEngineerEvent,
    ReadFileToolEvent,
    RunCommandToolEvent,
    SimulationRequestEvent,
    SkillReadEvent,
    WriteFileToolEvent,
)
from shared.simulation.schemas import SimulatorBackendType
from worker.api.schema import EditOp


class RemoteFilesystemMiddleware:
    """
    Middleware that proxies filesystem operations to a remote Worker,
    with durable execution via Temporal.
    """

    def __init__(self, client: WorkerClient, temporal_client: Client | None = None):
        self.client = client
        self.temporal_client = temporal_client
        self.read_only_paths = ["skills/", "utils/", "reviews/"]

    def _is_read_only(self, path: str | Path) -> bool:
        """Check if a path is in a read-only directory."""
        p = Path(path).as_posix().lstrip("/")
        return any(p.startswith(ro_path) for ro_path in self.read_only_paths)

    async def list_files(self, path: str | Path = "/") -> list[dict[str, Any]]:
        """List files via the Worker client."""
        await record_events(
            episode_id=self.client.session_id,
            events=[LsFilesToolEvent(path=str(path))],
        )
        return await self.client.list_files(str(path))

    async def inspect_topology(
        self, target_id: str, script_path: str | Path = "script.py"
    ) -> dict[str, Any]:
        """Inspect topological features via the Worker client."""
        # We could add a specific event type for this if needed
        result = await self.client.inspect_topology(target_id, str(script_path))
        return result.model_dump()

    async def exists(self, path: str | Path) -> bool:
        """Check if a file exists via the Worker client."""
        return await self.client.exists(str(path))

    async def read_file(self, path: str | Path) -> str:
        """Read file via the Worker client."""
        p_str = str(path).lstrip("/")
        events = [ReadFileToolEvent(path=p_str)]
        if p_str.startswith("skills/"):
            skill_name = p_str.split("/")[1] if "/" in p_str[7:] else p_str[7:]
            # Simple heuristic for skill name
            events.append(SkillReadEvent(skill_path=path, skill_name=skill_name))

        await record_events(
            episode_id=self.client.session_id,
            events=events,
        )

        if p_str.startswith(("skills/", "utils/")):
            module_name = p_str.split("/")[1] if "/" in p_str[7:] else p_str[7:]
            await record_events(
                episode_id=self.client.session_id,
                events=[
                    LibraryUsageEvent(
                        module_name=module_name, usage_type="reused", path=p_str
                    )
                ],
            )

        return await self.client.read_file(str(path))

    async def write_file(
        self, path: str | Path, content: str, overwrite: bool = True
    ) -> bool:
        """Write file via the Worker client, enforcing read-only constraints."""
        p_str = str(path)
        if self._is_read_only(p_str):
            raise PermissionError(f"Path '{p_str}' is read-only.")

        await record_events(
            episode_id=self.client.session_id,
            events=[
                WriteFileToolEvent(
                    path=p_str, content_snippet=content[:100], overwrite=overwrite
                )
            ],
        )
        success = await self.client.write_file(p_str, content, overwrite=overwrite)

        if success:
            # Track library usage (new)
            if path.lstrip("/").startswith(("skills/", "utils/")):
                module_name = (
                    path.lstrip("/").split("/")[1]
                    if "/" in path.lstrip("/")[7:]
                    else path.lstrip("/")[7:]
                )
                await record_events(
                    episode_id=self.client.session_id,
                    events=[
                        LibraryUsageEvent(
                            module_name=module_name, usage_type="new", path=path
                        )
                    ],
                )

            # Broadcast update and sync asset via helper
            await broadcast_file_update(self.client.session_id, p_str, content)

        return success

    async def edit_file(self, path: str | Path, edits: list[EditOp]) -> bool:
        """Edit a file via the Worker client."""
        p_str = str(path)
        if self._is_read_only(p_str):
            raise PermissionError(f"Path '{p_str}' is read-only.")

        await record_events(
            episode_id=self.client.session_id,
            events=[EditFileToolEvent(path=p_str, num_edits=len(edits))],
        )
        success = await self.client.edit_file(p_str, edits)
        if success:
            # For edits, we read the file back to get the updated
            # content for the Asset table
            try:
                content = await self.client.read_file(p_str)
                await sync_file_asset(self.client.session_id, p_str, content)
            except Exception:
                # Don't fail the edit if sync fails
                pass
        return success

    async def run_command(self, code: str, timeout: int = 30) -> dict[str, Any]:
        """
        Execute a command (Python code) via the Worker client,
        wrapped in Temporal for durability.
        """
        await record_events(
            episode_id=self.client.session_id,
            events=[RunCommandToolEvent(command=code)],
        )
        if self.temporal_client:
            # Wrap in Temporal workflow for durability
            return await self.temporal_client.execute_workflow(
                ScriptExecutionWorkflow.run,
                {
                    "code": code,
                    "session_id": self.client.session_id,
                    "timeout": timeout,
                },
                id=f"exec-{self.client.session_id}-{hash(code) % 10**8}",
                task_queue="simulation-task-queue",
            )
        # Fallback to direct client call if Temporal is not available
        result = await self.client.execute_python(code, timeout=timeout)
        return result.model_dump()

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> dict[str, Any]:
        return await self.run_command(code, timeout=timeout)

    async def grep(
        self, pattern: str, path: str | Path | None = None, glob: str | None = None
    ) -> list[dict[str, Any]]:
        """Search for a pattern in files via the Worker client."""
        p_str = str(path) if path else None
        await record_events(
            episode_id=self.client.session_id,
            events=[GrepToolEvent(pattern=pattern, path=p_str, glob=glob)],
        )
        results = await self.client.grep(pattern, path=p_str, glob=glob)
        return [match.model_dump() for match in results]

    async def simulate(
        self,
        script_path: str | Path,
        backend: SimulatorBackendType = SimulatorBackendType.GENESIS,
    ) -> dict[str, Any]:
        """Trigger physics simulation via worker client (with bundling)."""
        p_str = str(script_path)
        # Record request
        await record_events(
            episode_id=self.client.session_id,
            events=[SimulationRequestEvent(script_path=p_str)],
        )

        if self.temporal_client:
            # Bundle from light worker
            bundle = await self.client.bundle_session()

            # Execute via Temporal
            res = await self.temporal_client.execute_workflow(
                HeavySimulationWorkflow.run,
                {
                    "bundle_bytes": bundle,
                    "script_path": p_str,
                    "backend": backend.value,
                    "smoke_test_mode": False,
                    "session_id": self.client.session_id,
                },
                id=f"sim-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
            await record_simulation_result(self.client.session_id, res)
            return res

        result = await self.client.simulate(p_str, backend=backend)
        res_dict = result.model_dump()

        # Record result and detect instability via helper
        await record_simulation_result(self.client.session_id, res_dict)

        return res_dict

    async def preview(
        self,
        script_path: str | Path,
        pitch: float = -45.0,
        yaw: float = 45.0,
    ) -> dict[str, Any]:
        """Trigger design preview via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            return await self.temporal_client.execute_workflow(
                HeavyPreviewWorkflow.run,
                {
                    "bundle_bytes": bundle,
                    "script_path": p_str,
                    "pitch": pitch,
                    "yaw": yaw,
                },
                id=f"prev-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )

        return await self.client.preview(p_str, pitch=pitch, yaw=yaw)

    async def validate(self, script_path: str | Path) -> dict[str, Any]:
        """Trigger geometric validation via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res_dict = await self.temporal_client.execute_workflow(
                HeavyValidationWorkflow.run,
                {
                    "bundle_bytes": bundle,
                    "script_path": p_str,
                    "session_id": self.client.session_id,
                },
                id=f"val-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
        else:
            result = await self.client.validate(p_str)
            res_dict = result.model_dump()

        # Record as ManufacturabilityCheckEvent
        await record_events(
            episode_id=self.client.session_id,
            events=[
                ManufacturabilityCheckEvent(
                    part_id=p_str,  # Using script_path as part identifier
                    method="auto_geometric",
                    result=res_dict.get("success", False),
                    price=res_dict.get("price"),
                    weight_g=res_dict.get("weight_g"),
                    metadata=res_dict,
                )
            ],
        )

        return res_dict

    async def submit(self, script_path: str | Path) -> dict[str, Any]:
        """Trigger handover to review via worker client."""
        p_str = str(script_path)
        result = await self.client.submit(p_str)
        res_dict = result.model_dump()

        # Record as PlanSubmissionEngineerEvent
        await record_events(
            episode_id=self.client.session_id,
            events=[
                PlanSubmissionEngineerEvent(
                    plan_path=p_str,
                )
            ],
        )

        return res_dict
