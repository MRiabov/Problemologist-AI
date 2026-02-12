from typing import Literal

import httpx

from worker.api.schema import (
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GitCommitResponse,
    GitStatusResponse,
    GrepMatch,
)
from worker.filesystem.backend import FileInfo
from worker.workbenches.models import ManufacturingMethod, WorkbenchResult


class WorkerClient:
    """Async client for interacting with the Problemologist Worker API."""

    def __init__(
        self,
        base_url: str,
        session_id: str,
        http_client: httpx.AsyncClient | None = None,
    ):
        """Initialize the worker client.

        Args:
            base_url: Base URL of the worker service (e.g., http://worker:8000).
            session_id: Session ID to use for all requests.
            http_client: Pre-configured httpx.AsyncClient to reuse.
        """
        self.base_url = base_url.rstrip("/")
        self.session_id = session_id
        self.headers = {"X-Session-ID": session_id}
        self.http_client = http_client

    async def _get_client(self) -> httpx.AsyncClient:
        """Helper to get a client, either shared or new (for compatibility)."""
        if self.http_client:
            return self.http_client
        # Fallback for code not yet updated to provide a client
        return httpx.AsyncClient()

    async def _close_client(self, client: httpx.AsyncClient):
        """Helper to close a client only if it was created locally."""
        if not self.http_client:
            await client.aclose()

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
            response.raise_for_status()
            return response.json()["content"]
        finally:
            await self._close_client(client)

    async def write_file(self, path: str, content: str) -> bool:
        """Write content to a file."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/write",
                json={"path": path, "content": content},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"
        finally:
            await self._close_client(client)

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
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger physics simulation via worker."""
        client = await self._get_client()
        try:
            payload = {"script_path": script_path}
            if script_content is not None:
                payload["script_content"] = script_content
            response = await client.post(
                f"{self.base_url}/benchmark/simulate",
                json=payload,
                headers=self.headers,
                timeout=60.0,
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
            response = await client.post(
                f"{self.base_url}/benchmark/validate",
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
    ) -> WorkbenchResult:
        """Trigger manufacturing analysis via worker."""
        client = await self._get_client()
        try:
            payload = {"script_path": script_path, "method": method}
            if script_content is not None:
                payload["script_content"] = script_content
            response = await client.post(
                f"{self.base_url}/benchmark/analyze",
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
            response = await client.post(
                f"{self.base_url}/benchmark/submit",
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
