import logging
from pathlib import Path

from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.observability.middleware_helper import (
    broadcast_file_update,
    record_events,
    record_simulation_result,
)
from controller.workflows.execution import ScriptExecutionWorkflow
from controller.workflows.heavy import (
    HeavyPreviewWorkflow,
    HeavyRenderSnapshotWorkflow,
    HeavySimulationWorkflow,
    HeavyValidationWorkflow,
)
from shared.models.simulation import SimulationResult
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
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import (
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GrepMatch,
    HeavyPreviewParams,
    HeavyPreviewResponse,
    HeavyRenderSnapshotParams,
    HeavyRenderSnapshotResponse,
    HeavySimulationParams,
    HeavyValidationParams,
    HeavyValidationResponse,
    InspectTopologyResponse,
    PreviewDesignResponse,
    RenderSnapshotResponse,
    ScriptExecutionRequest,
)

logger = logging.getLogger(__name__)


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

    async def list_files(self, path: str | Path = "/") -> list[FileInfo]:
        """List files via the Worker client."""
        await record_events(
            episode_id=self.client.session_id,
            events=[LsFilesToolEvent(path=str(path))],
        )
        return await self.client.list_files(str(path))

    async def inspect_topology(
        self, target_id: str, script_path: str | Path = "script.py"
    ) -> InspectTopologyResponse:
        """Inspect topological features via the Worker client."""
        # We could add a specific event type for this if needed
        return await self.client.inspect_topology(target_id, str(script_path))

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
                await broadcast_file_update(self.client.session_id, p_str, content)
            except Exception:
                # Don't fail the edit if sync fails
                pass
        return success

    async def run_command(self, code: str, timeout: int = 30) -> ExecuteResponse:
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
                ScriptExecutionRequest(
                    code=code,
                    session_id=self.client.session_id,
                    timeout=timeout,
                ),
                id=f"exec-{self.client.session_id}-{hash(code) % 10**8}",
                task_queue="simulation-task-queue",
            )
        # Fallback to direct client call if Temporal is not available
        return await self.client.execute_python(code, timeout=timeout)

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> ExecuteResponse:
        return await self.run_command(code, timeout=timeout)

    async def grep(
        self, pattern: str, path: str | Path | None = None, glob: str | None = None
    ) -> list[GrepMatch]:
        """Search for a pattern in files via the Worker client."""
        p_str = str(path) if path else None
        await record_events(
            episode_id=self.client.session_id,
            events=[GrepToolEvent(pattern=pattern, path=p_str, glob=glob)],
        )
        return await self.client.grep(pattern, path=p_str, glob=glob)

    async def simulate(
        self,
        script_path: str | Path,
        backend: SimulatorBackendType = SimulatorBackendType.GENESIS,
    ) -> BenchmarkToolResponse | SimulationResult:
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
                HeavySimulationParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    backend=backend,
                    smoke_test_mode=False,
                    session_id=self.client.session_id,
                ),
                id=f"sim-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
            await record_simulation_result(self.client.session_id, res)
            return res

        return await self.client.simulate(p_str, backend=backend)

    async def preview(
        self,
        script_path: str | Path,
        pitch: float = -45.0,
        yaw: float = 45.0,
    ) -> PreviewDesignResponse:
        """Trigger design preview via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res: HeavyPreviewResponse = await self.temporal_client.execute_workflow(
                HeavyPreviewWorkflow.run,
                HeavyPreviewParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    pitch=pitch,
                    yaw=yaw,
                ),
                id=f"prev-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
            return PreviewDesignResponse(
                success=res.success,
                message="Preview generated via Temporal"
                if res.success
                else "Preview failed",
                image_path=res.filename,
            )

        return await self.client.preview(p_str, pitch=pitch, yaw=yaw)

    async def render_snapshot(
        self,
        target_ids: list[str],
        view_matrix: list[list[float]],
        script_path: str | Path = "script.py",
    ) -> RenderSnapshotResponse:
        """Render a snapshot of selected features via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res: HeavyRenderSnapshotResponse = (
                await self.temporal_client.execute_workflow(
                    HeavyRenderSnapshotWorkflow.run,
                    HeavyRenderSnapshotParams(
                        bundle_bytes=bundle,
                        script_path=p_str,
                        target_ids=target_ids,
                        view_matrix=view_matrix,
                    ),
                    id=f"snap-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                    task_queue="simulation-task-queue",
                )
            )
            return RenderSnapshotResponse(
                success=res.success,
                message="Snapshot generated via Temporal"
                if res.success
                else f"Snapshot failed: {res.error}",
                image_key=res.image_key,
            )

        return await self.client.render_snapshot(
            target_ids=target_ids,
            view_matrix=view_matrix,
            script_path=p_str,
        )

    async def validate(
        self, script_path: str | Path
    ) -> BenchmarkToolResponse | HeavyValidationResponse:
        """Trigger geometric validation via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res = await self.temporal_client.execute_workflow(
                HeavyValidationWorkflow.run,
                HeavyValidationParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    session_id=self.client.session_id,
                ),
                id=f"val-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
        else:
            res = await self.client.validate(p_str)

        await record_events(
            episode_id=self.client.session_id,
            events=[
                ManufacturabilityCheckEvent(
                    part_id=p_str,  # Using script_path as part identifier
                    method="auto_geometric",
                    result=res.success,
                    price=getattr(res, "price", None),
                    weight_g=getattr(res, "weight_g", None),
                    metadata=res.model_dump(),
                )
            ],
        )

        return res

    async def submit(self, script_path: str | Path) -> BenchmarkToolResponse:
        """Trigger handover to review via worker client."""
        p_str = str(script_path)
        res = await self.client.submit(p_str)

        # Record as PlanSubmissionEngineerEvent
        await record_events(
            episode_id=self.client.session_id,
            events=[
                PlanSubmissionEngineerEvent(
                    plan_path=p_str,
                )
            ],
        )

        return res
