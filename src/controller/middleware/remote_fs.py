from typing import Any
from src.controller.clients.worker import WorkerClient


class RemoteFilesystemMiddleware:
    """Middleware that proxies filesystem operations to a remote Worker."""

    def __init__(self, client: WorkerClient):
        self.client = client
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
        """Execute a command (Python code) via the Worker client."""
        # For now, we assume 'run_command' means executing Python code
        # as per Worker API.
        return await self.client.execute_python(code, timeout=timeout)

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> dict[str, Any]:
        return await self.run_command(code, timeout=timeout)
