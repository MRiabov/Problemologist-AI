from typing import Any

import structlog
import httpx
from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.observability.tracing import record_worker_events, sync_asset
from controller.workflows.execution import ScriptExecutionWorkflow
from shared.simulation.schemas import SimulatorBackendType
from shared.observability.schemas import (
    EditFileToolEvent,
    GrepToolEvent,
    LibraryUsageEvent,
    LsFilesToolEvent,
    ManufacturabilityCheckEvent,
    PlanSubmissionEngineerEvent,
    ReadFileToolEvent,
    RunCommandToolEvent,
    SimulationFailureReason,
    SimulationInstabilityEvent,
    SimulationRequestEvent,
    SimulationResultEvent,
    SkillReadEvent,
    WriteFileToolEvent,
)
from worker.api.schema import EditOp


class RemoteFilesystemMiddleware:
    """Middleware that proxies filesystem operations to a remote Worker, with durable execution via Temporal."""

    def __init__(self, client: WorkerClient, temporal_client: Client | None = None):
        self.client = client
        self.temporal_client = temporal_client
        self.read_only_paths = ["skills/", "utils/", "reviews/"]

    def _is_read_only(self, path: str) -> bool:
        """Check if a path is in a read-only directory."""
        path = path.lstrip("/")
        return any(path.startswith(ro_path) for ro_path in self.read_only_paths)

    async def list_files(self, path: str = "/") -> list[dict[str, Any]]:
        """List files via the Worker client."""
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[LsFilesToolEvent(path=path)],
        )
        return await self.client.list_files(path)

    async def inspect_topology(
        self, target_id: str, script_path: str = "script.py"
    ) -> dict[str, Any]:
        """Inspect topological features via the Worker client."""
        # We could add a specific event type for this if needed
        result = await self.client.inspect_topology(target_id, script_path)
        return result.model_dump()

    async def exists(self, path: str) -> bool:
        """Check if a file exists via the Worker client."""
        return await self.client.exists(path)

    async def read_file(self, path: str) -> str:
        """Read file via the Worker client."""
        events = [ReadFileToolEvent(path=path)]
        if path.lstrip("/").startswith("skills/"):
            skill_name = (
                path.lstrip("/").split("/")[1]
                if "/" in path.lstrip("/")[7:]
                else path.lstrip("/")[7:]
            )
            # Simple heuristic for skill name
            events.append(SkillReadEvent(skill_path=path, skill_name=skill_name))

        await record_worker_events(
            episode_id=self.client.session_id,
            events=events,
        )

        # Track library usage (reused)
        if path.lstrip("/").startswith(("skills/", "utils/")):
            module_name = (
                path.lstrip("/").split("/")[1]
                if "/" in path.lstrip("/")[7:]
                else path.lstrip("/")[7:]
            )
            await record_worker_events(
                episode_id=self.client.session_id,
                events=[
                    LibraryUsageEvent(
                        module_name=module_name, usage_type="reused", path=path
                    )
                ],
            )

        return await self.client.read_file(path)

    async def write_file(self, path: str, content: str, overwrite: bool = True) -> bool:
        """Write file via the Worker client, enforcing read-only constraints."""
        if self._is_read_only(path):
            raise PermissionError(f"Path '{path}' is read-only.")

        await record_worker_events(
            episode_id=self.client.session_id,
            events=[
                WriteFileToolEvent(
                    path=path, content_snippet=content[:100], overwrite=overwrite
                )
            ],
        )
        success = await self.client.write_file(path, content, overwrite=overwrite)

        if success:
            # Track library usage (new)
            if path.lstrip("/").startswith(("skills/", "utils/")):
                module_name = (
                    path.lstrip("/").split("/")[1]
                    if "/" in path.lstrip("/")[7:]
                    else path.lstrip("/")[7:]
                )
                await record_worker_events(
                    episode_id=self.client.session_id,
                    events=[
                        LibraryUsageEvent(
                            module_name=module_name, usage_type="new", path=path
                        )
                    ],
                )

            import uuid
            from datetime import datetime

            from controller.observability.broadcast import EpisodeBroadcaster

            # Broadcast update to frontend via unified event hub
            try:
                episode_id = uuid.UUID(self.client.session_id)
                await EpisodeBroadcaster.get_instance().broadcast(
                    episode_id,
                    {
                        "type": "file_update",
                        "data": {"path": path, "content": content},
                        "timestamp": datetime.now(datetime.UTC).isoformat(),
                    },
                )
                # Sync to Asset table for Explorer visibility
                await sync_asset(episode_id, path, content)
            except ValueError:
                # If session_id is not a UUID, we can't broadcast (standard in some dev/test setups)
                pass
            except Exception:
                # Don't fail the write operation if broadcast fails
                pass

        return success

    async def edit_file(self, path: str, edits: list[EditOp]) -> bool:
        """Edit a file via the Worker client."""
        if self._is_read_only(path):
            raise PermissionError(f"Path '{path}' is read-only.")

        await record_worker_events(
            episode_id=self.client.session_id,
            events=[EditFileToolEvent(path=path, num_edits=len(edits))],
        )
        success = await self.client.edit_file(path, edits)
        if success:
            # For edits, we read the file back to get the updated content for the Asset table
            try:
                content = await self.client.read_file(path)
                await sync_asset(self.client.session_id, path, content)
            except Exception:
                # Don't fail the edit if sync fails
                pass
        return success

    async def run_command(self, code: str, timeout: int = 30) -> dict[str, Any]:
        """Execute a command (Python code) via the Worker client, wrapped in Temporal for durability."""
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[RunCommandToolEvent(command=code)],
        )
        if self.temporal_client:
            # Wrap in Temporal workflow for durability
            result = await self.temporal_client.execute_workflow(
                ScriptExecutionWorkflow.run,
                {
                    "code": code,
                    "session_id": self.client.session_id,
                    "timeout": timeout,
                },
                id=f"exec-{self.client.session_id}-{hash(code) % 10**8}",
                task_queue="simulation-task-queue",
            )
            return result
        # Fallback to direct client call if Temporal is not available
        result = await self.client.execute_python(code, timeout=timeout)
        return result.model_dump()

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> dict[str, Any]:
        return await self.run_command(code, timeout=timeout)

    async def grep(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[dict[str, Any]]:
        """Search for a pattern in files via the Worker client."""
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[GrepToolEvent(pattern=pattern, path=path, glob=glob)],
        )
        results = await self.client.grep(pattern, path=path, glob=glob)
        return [match.model_dump() for match in results]

    async def simulate(
        self,
        script_path: str,
        backend: SimulatorBackendType = SimulatorBackendType.MUJOCO,
    ) -> dict[str, Any]:
        """Trigger physics simulation via worker client."""
        # Record request
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[SimulationRequestEvent(script_path=script_path)],
        )

        result = await self.client.simulate(script_path, backend=backend)
        res_dict = result.model_dump()

        # Record result
        # Map failure reason if present in metadata or result
        failure_reason = SimulationFailureReason.NONE
        if not res_dict.get("success", True):
            # This is a guestimate mapping for now
            raw_reason = res_dict.get("failure_reason", "").lower()
            if "timeout" in raw_reason:
                failure_reason = SimulationFailureReason.TIMEOUT
            elif "out of bounds" in raw_reason:
                failure_reason = SimulationFailureReason.OUT_OF_BOUNDS
            elif "forbid" in raw_reason:
                failure_reason = SimulationFailureReason.FORBID_ZONE_HIT
            elif "break" in raw_reason or "stress" in raw_reason:
                failure_reason = SimulationFailureReason.PART_BREAKAGE

        await record_worker_events(
            episode_id=self.client.session_id,
            events=[
                SimulationResultEvent(
                    success=res_dict.get("success", True),
                    failure_reason=failure_reason,
                    time_elapsed_s=res_dict.get("time_elapsed_s", 0.0),
                    compute_time_ms=res_dict.get("compute_time_ms", 0.0),
                    metadata=res_dict,
                )
            ],
        )

        # Detect instability
        if not res_dict.get("success", True):
            raw_reason = res_dict.get("failure_reason", "").lower()
            if any(
                word in raw_reason
                for word in ["nan", "penetration", "instability", "joint violation"]
            ):
                instability_type = "unknown"
                if "nan" in raw_reason:
                    instability_type = "nan"
                elif "penetration" in raw_reason:
                    instability_type = "penetration"
                elif "joint violation" in raw_reason:
                    instability_type = "joint_violation"

                await record_worker_events(
                    episode_id=self.client.session_id,
                    events=[
                        SimulationInstabilityEvent(
                            instability_type=instability_type,
                            part_ids=res_dict.get("offending_parts", []),
                            message=res_dict.get("failure_reason"),
                        )
                    ],
                )

        return res_dict

    async def validate(self, script_path: str) -> dict[str, Any]:
        """Trigger geometric validation via worker client."""
        result = await self.client.validate(script_path)
        res_dict = result.model_dump()

        # Record as ManufacturabilityCheckEvent
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[
                ManufacturabilityCheckEvent(
                    part_id=script_path,  # Using script_path as part identifier if checking one
                    method="auto_geometric",
                    result=res_dict.get("success", False),
                    price=res_dict.get("price"),
                    weight_g=res_dict.get("weight_g"),
                    metadata=res_dict,
                )
            ],
        )

        return res_dict

    async def submit(self, script_path: str) -> dict[str, Any]:
        """Trigger handover to review via worker client."""
        result = await self.client.submit(script_path)
        res_dict = result.model_dump()

        # Record as PlanSubmissionEngineerEvent
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[
                PlanSubmissionEngineerEvent(
                    plan_path=script_path,
                )
            ],
        )

        return res_dict
