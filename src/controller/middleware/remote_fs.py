from typing import Any, Optional
from temporalio.client import Client
from src.controller.clients.worker import WorkerClient
from src.controller.workflows.execution import ScriptExecutionWorkflow


class RemoteFilesystemMiddleware:
    """Middleware that proxies filesystem operations to a remote Worker, with durable execution via Temporal."""

    def __init__(self, client: WorkerClient, temporal_client: Optional[Client] = None):
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
        return await self.client.write_file(path, content)

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
        else:
            # Fallback to direct client call if Temporal is not available
            result = await self.client.execute_python(code, timeout=timeout)
            return result.model_dump()

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> dict[str, Any]:
        return await self.run_command(code, timeout=timeout)

    async def simulate(self, script_path: str = "script.py") -> dict[str, Any]:
        """Trigger physics simulation via worker client."""
        result = await self.client.simulate(script_path)
        return result.model_dump()

    async def validate(self, script_path: str = "script.py") -> dict[str, Any]:
        """Trigger geometric validation via worker client."""
        result = await self.client.validate(script_path)
        return result.model_dump()

    async def submit(self, script_path: str = "script.py") -> dict[str, Any]:
        """Trigger handover to review via worker client."""
        result = await self.client.submit(script_path)
        return result.model_dump()