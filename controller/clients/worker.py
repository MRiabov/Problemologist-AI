import httpx

from worker.api.schema import (
    AnalyzeRequest,
    AnalyzeResponse,
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GitCommitResponse,
    GlobRequest,
    GrepMatch,
)
from worker.filesystem.backend import FileInfo


class WorkerClient:
    """Async client for interacting with the Problemologist Worker API."""

    def __init__(self, base_url: str, session_id: str):
        """Initialize the worker client.

        Args:
            base_url: Base URL of the worker service (e.g., http://worker:8000).
            session_id: Session ID to use for all requests.
        """
        self.base_url = base_url.rstrip("/")
        self.session_id = session_id
        self.headers = {"X-Session-ID": session_id}

    async def list_files(self, path: str = "/") -> list[FileInfo]:
        """List contents of a directory."""
        # Ensure path is not empty to avoid 422 errors from Pydantic min_length=1
        if not path:
            path = "/"
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/ls",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return [FileInfo.model_validate(item) for item in response.json()]

    async def grep(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch]:
        """Search for a pattern in files."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/grep",
                json={"pattern": pattern, "path": path, "glob": glob},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return [GrepMatch.model_validate(item) for item in response.json()]

    async def glob(self, pattern: str, path: str = "/") -> list[FileInfo]:
        """Find files matching a glob pattern."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/glob",
                json={"pattern": pattern, "path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return [FileInfo.model_validate(item) for item in response.json()]

    async def read_file(self, path: str) -> str:
        """Read file contents."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/read",
                json={"path": path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["content"]

    async def write_file(self, path: str, content: str) -> bool:
        """Write content to a file."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/write",
                json={"path": path, "content": content},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

    async def edit_file(self, path: str, edits: list[EditOp]) -> bool:
        """Edit a file with one or more operations.

        Args:
            path: Virtual file path.
            edits: List of EditOp objects.

        Returns:
            True if successful.
        """
        async with httpx.AsyncClient() as client:
            # Pydantic models need to be converted to dict for json serialization
            json_edits = [op.model_dump() for op in edits]
            response = await client.post(
                f"{self.base_url}/fs/edit",
                json={"path": path, "edits": json_edits},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

    async def upload_files(self, files: list[tuple[str, bytes]]) -> bool:
        """Upload multiple files to the workspace."""
        multipart_files = []
        for path, content in files:
            # Field name 'files', filename=path, content=content
            multipart_files.append(("files", (path, content)))

        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/upload_file",
                files=multipart_files,
                headers=self.headers,
                timeout=60.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

    async def download_file(self, path: str) -> bytes:
        """Download a file as bytes."""
        # Note: We rely on httpx handling URL encoding for the path if needed,
        # but for clean paths, direct injection works with FastAPI's {path:path}.
        async with httpx.AsyncClient() as client:
            response = await client.get(
                f"{self.base_url}/assets/{path}",
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return response.content

    async def execute_python(self, code: str, timeout: int = 30) -> ExecuteResponse:
        """Execute Python code in the sandboxed runtime."""
        async with httpx.AsyncClient() as client:
            http_timeout = float(timeout) + 5.0
            response = await client.post(
                f"{self.base_url}/runtime/execute",
                json={"code": code, "timeout": timeout},
                headers=self.headers,
                timeout=http_timeout,
            )
            response.raise_for_status()
            return ExecuteResponse.model_validate(response.json())

    async def simulate(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger physics simulation via worker."""
        async with httpx.AsyncClient() as client:
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

    async def validate(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger geometric validation via worker."""
        async with httpx.AsyncClient() as client:
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

    async def submit(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger handover to review via worker."""
        async with httpx.AsyncClient() as client:
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

    async def analyze(self, request: AnalyzeRequest) -> AnalyzeResponse:
        """Analyze manufacturability of a part."""
        async with httpx.AsyncClient() as client:
            json_body = request.model_dump(mode="json")
            response = await client.post(
                f"{self.base_url}/benchmark/analyze",
                json=json_body,
                headers=self.headers,
                timeout=60.0,
            )
            response.raise_for_status()
            return AnalyzeResponse.model_validate(response.json())

    async def get_health(self) -> dict[str, str]:
        """Check the health of the worker service."""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{self.base_url}/health", timeout=5.0)
            response.raise_for_status()
            return response.json()

    async def git_init(self) -> bool:
        """Initialize a git repository in the workspace."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/git/init",
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

    async def git_commit(self, message: str) -> GitCommitResponse:
        """Commit changes and sync to S3."""
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/git/commit",
                json={"message": message},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return GitCommitResponse.model_validate(response.json())
