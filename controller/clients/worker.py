import asyncio
import base64
from typing import Any, Literal

import httpx

from shared.simulation.schemas import SimulatorBackendType
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import (
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GitCommitResponse,
    GitStatusResponse,
    GrepMatch,
    InspectTopologyResponse,
)
from shared.workers.workbench_models import ManufacturingMethod, WorkbenchResult


class WorkerClient:
    """Async client for interacting with the Problemologist Worker API."""

    def __init__(
        self,
        base_url: str,
        session_id: str,
        http_client: httpx.AsyncClient | None = None,
        heavy_url: str | None = None,
    ):
        """Initialize the worker client.

        Args:
            base_url: Base URL of the light worker service (e.g., http://worker-light:8000).
            session_id: Session ID to use for all requests.
            http_client: Pre-configured httpx.AsyncClient to reuse.
            heavy_url: Optional base URL of the heavy worker service.
        """
        self.base_url = base_url.rstrip("/")
        self.light_url = self.base_url
        self.heavy_url = (heavy_url or base_url).rstrip("/")
        self.session_id = session_id
        self.headers = {"X-Session-ID": session_id}
        self.http_client = http_client
        self._loop_clients: dict[int, httpx.AsyncClient] = {}
        self._loop_locks: dict[int, asyncio.Lock] = {}

    async def _get_client(self) -> httpx.AsyncClient:
        """Helper to get a client, either shared or new (for compatibility)."""
        if self.http_client:
            return self.http_client

        loop = asyncio.get_running_loop()
        loop_id = id(loop)

        if loop_id not in self._loop_clients:
            # Note: Minimal race condition risk here across threads due to GIL and loop-local id,
            # but even if two threads create different clients for different loops, it's fine.
            if loop_id not in self._loop_locks:
                self._loop_locks[loop_id] = asyncio.Lock()

            async with self._loop_locks[loop_id]:
                if loop_id not in self._loop_clients:
                    self._loop_clients[loop_id] = httpx.AsyncClient()

        return self._loop_clients[loop_id]

    async def _close_client(self, client: httpx.AsyncClient):
        """Helper to close a client only if it was created locally."""
        # We no longer close the client here to allow connection pooling across requests.
        # The client will be closed when aclose() is called or the instance is destroyed.
        pass

    async def aclose(self):
        """Explicitly close all HTTP clients created for different loops."""
        # WP08: Handle loop-specific clients
        for loop_id, client in list(self._loop_clients.items()):
            await client.aclose()
            del self._loop_clients[loop_id]

        # Compatibility for legacy _cached_client check in tests
        if hasattr(self, "_cached_client") and self._cached_client:
            await self._cached_client.aclose()
            self._cached_client = None

    async def inspect_topology(
        self, target_id: str, script_path: str = "script.py"
    ) -> InspectTopologyResponse:
        """Inspect topological features via worker."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/topology/inspect",
                json={"target_id": target_id, "script_path": script_path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return InspectTopologyResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def list_files(self, path: str = "/") -> list[FileInfo]:
        """List contents of a directory."""
        if not path:
            path = "/"
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/ls",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return [FileInfo.model_validate(item) for item in response.json()]
        finally:
            await self._close_client(client)

    async def grep(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch]:
        """Search for a pattern in files."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/grep",
                json={"pattern": pattern, "path": path, "glob": glob},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return [GrepMatch.model_validate(item) for item in response.json()]
        finally:
            await self._close_client(client)

    async def read_file(self, path: str) -> str:
        """Read file contents."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/read",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            if response.status_code == 404:
                return f"Error: File '{path}' not found."
            response.raise_for_status()
            return response.json()["content"]
        finally:
            await self._close_client(client)

    async def exists(self, path: str) -> bool:
        """Check if a file exists."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/exists",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            if response.status_code == 404:
                return False
            response.raise_for_status()
            return response.json()["exists"]
        except Exception:
            return False
        finally:
            await self._close_client(client)

    async def write_file(self, path: str, content: str, overwrite: bool = True) -> bool:
        """Write content to a file."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/write",
                json={"path": path, "content": content, "overwrite": overwrite},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def delete_file(self, path: str) -> bool:
        """Delete a file or directory."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/delete",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def move_file(self, source: str, destination: str) -> bool:
        """Move or rename a file or directory."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/move",
                json={"source": source, "destination": destination},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def copy_file(self, source: str, destination: str) -> bool:
        """Copy a file or directory."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/copy",
                json={"source": source, "destination": destination},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def mkdir(self, path: str) -> bool:
        """Create a directory."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/mkdir",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def bundle_session(self) -> bytes:
        """Bundle the session workspace into a gzipped tarball from the light worker."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.light_url}/fs/bundle",
                headers=self.headers,
                timeout=60.0,
            )
            response.raise_for_status()
            return response.content
        finally:
            await self._close_client(client)

    async def _add_bundle_to_payload(self, payload: dict):
        """Append a workspace bundle to the payload if light and heavy workers are different."""
        if self.light_url != self.heavy_url:
            bundle = await self.bundle_session()
            payload["bundle_base64"] = base64.b64encode(bundle).decode("utf-8")

    async def upload_file(self, path: str, content: bytes) -> bool:
        """Upload a file with binary content."""
        client = await self._get_client()
        try:
            # Prepare multipart form data
            files = {"file": ("blob", content)}
            data = {"path": path}
            response = await client.post(
                f"{self.base_url}/fs/upload_file",
                data=data,
                files=files,
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def read_file_binary(self, path: str) -> bytes:
        """Read file contents as binary."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/read_blob",
                json={"path": path},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return response.content
        finally:
            await self._close_client(client)

    async def preview(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        pitch: float = -45.0,
        yaw: float = 45.0,
    ) -> dict[str, Any]:
        """Trigger design preview via worker."""
        client = await self._get_client()
        try:
            payload = {
                "script_path": script_path,
                "pitch": pitch,
                "yaw": yaw,
            }
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/preview",
                json=payload,
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return response.json()
        finally:
            await self._close_client(client)

    async def edit_file(self, path: str, edits: list[EditOp]) -> bool:
        """Edit a file with one or more operations."""
        client = await self._get_client()
        try:
            json_edits = [op.model_dump() for op in edits]
            response = await client.post(
                f"{self.base_url}/fs/edit",
                json={"path": path, "edits": json_edits},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def execute_python(self, code: str, timeout: int = 30) -> ExecuteResponse:
        """Execute Python code in the sandboxed runtime."""
        client = await self._get_client()
        try:
            http_timeout = float(timeout) + 5.0
            response = await client.post(
                f"{self.base_url}/runtime/execute",
                json={"code": code, "timeout": timeout},
                headers=self.headers,
                timeout=http_timeout,
            )
            response.raise_for_status()
            return ExecuteResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def simulate(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        backend: SimulatorBackendType = SimulatorBackendType.GENESIS,
    ) -> BenchmarkToolResponse:
        """Trigger physics simulation via worker."""
        client = await self._get_client()
        try:
            payload = {"script_path": script_path, "backend": backend}
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/simulate",
                json=payload,
                headers=self.headers,
                timeout=600.0,
            )
            response.raise_for_status()
            return BenchmarkToolResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def validate(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger geometric validation via worker."""
        client = await self._get_client()
        try:
            payload = {"script_path": script_path}
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/validate",
                json=payload,
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return BenchmarkToolResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def analyze(
        self,
        method: ManufacturingMethod,
        script_path: str = "script.py",
        script_content: str | None = None,
        quantity: int = 1,
    ) -> WorkbenchResult:
        """Trigger manufacturing analysis via worker."""
        client = await self._get_client()
        try:
            payload = {
                "script_path": script_path,
                "method": method,
                "quantity": quantity,
            }
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/analyze",
                json=payload,
                headers=self.headers,
                timeout=60.0,
            )
            response.raise_for_status()
            return WorkbenchResult.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def submit(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger handover to review via worker."""
        client = await self._get_client()
        try:
            payload = {"script_path": script_path}
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/submit",
                json=payload,
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return BenchmarkToolResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def get_health(self) -> dict[str, str]:
        """Check the health of the worker service."""
        client = await self._get_client()
        try:
            response = await client.get(f"{self.base_url}/health", timeout=5.0)
            response.raise_for_status()
            return response.json()
        finally:
            await self._close_client(client)

    async def git_init(self) -> bool:
        """Initialize a git repository in the workspace."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/init",
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def git_commit(self, message: str) -> GitCommitResponse:
        """Commit changes and sync to S3."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/commit",
                json={"message": message},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return GitCommitResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def git_status(self) -> GitStatusResponse:
        """Get repository status."""
        client = await self._get_client()
        try:
            response = await client.get(
                f"{self.base_url}/git/status",
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return GitStatusResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def git_resolve(
        self, file_path: str, strategy: Literal["ours", "theirs"]
    ) -> bool:
        """Resolve a merge conflict."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/resolve",
                json={"file_path": file_path, "strategy": strategy},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def git_merge_abort(self) -> bool:
        """Abort a merge."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/merge/abort",
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

    async def git_merge_complete(self, message: str | None = None) -> GitCommitResponse:
        """Complete a merge."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/merge/complete",
                json={"message": message},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return GitCommitResponse.model_validate(response.json())
        finally:
            await self._close_client(client)
