from typing import Any

from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.workflows.execution import ScriptExecutionWorkflow
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import (
    SimulationRequestEvent,
    SimulationResultEvent,
    ManufacturabilityCheckEvent,
    PlanSubmissionEvent,
    SimulationFailureReason,
)


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
        return await self.client.list_files(path)

    async def read_file(self, path: str) -> str:
        """Read file via the Worker client."""
        return await self.client.read_file(path)

    async def write_file(self, path: str, content: str) -> bool:
        """Write file via the Worker client, enforcing read-only constraints."""
        if self._is_read_only(path):
            raise PermissionError(f"Path '{path}' is read-only.")

        success = await self.client.write_file(path, content)

        if success:
            from datetime import datetime
            from controller.observability.broadcast import EpisodeBroadcaster
            import uuid

            # Broadcast update to frontend via unified event hub
            try:
                episode_id = uuid.UUID(self.client.session_id)
                await EpisodeBroadcaster.get_instance().broadcast(
                    episode_id,
                    {
                        "type": "file_update",
                        "data": {"path": path, "content": content},
                        "timestamp": datetime.utcnow().isoformat(),
                    },
                )
            except ValueError:
                # If session_id is not a UUID, we can't broadcast (standard in some dev/test setups)
                pass
            except Exception as e:
                # Don't fail the write operation if broadcast fails
                pass

        return success

    async def run_command(self, code: str, timeout: int = 30) -> dict[str, Any]:
        """Execute a command (Python code) via the Worker client, wrapped in Temporal for durability."""
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

    async def simulate(self, script_path: str) -> dict[str, Any]:
        """Trigger physics simulation via worker client."""
        # Record request
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[SimulationRequestEvent(script_path=script_path)],
        )

        result = await self.client.simulate(script_path)
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

        # Record as PlanSubmissionEvent
        await record_worker_events(
            episode_id=self.client.session_id,
            events=[
                PlanSubmissionEvent(
                    source="engineer",  # Default to engineer in this middleware for now
                    plan_path=script_path,
                )
            ],
        )

        return res_dict
