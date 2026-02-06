import httpx
from typing import List, Optional, Dict, Any, Union
<<<<<<< HEAD
from src.worker.api.schema import ExecuteResponse
from src.worker.filesystem.backend import FileInfo

=======
>>>>>>> 001-agentic-cad-environment-WP03

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

<<<<<<< HEAD
    async def list_files(self, path: str = "/") -> List[FileInfo]:
=======
    async def list_files(self, path: str = "/") -> List[Dict[str, Any]]:
>>>>>>> 001-agentic-cad-environment-WP03
        """List contents of a directory.

        Args:
            path: Virtual directory path.

        Returns:
<<<<<<< HEAD
            List of FileInfo objects.
=======
            List of FileInfo dictionaries.
>>>>>>> 001-agentic-cad-environment-WP03
        """
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/ls",
                json={"path": path},
                headers=self.headers,
<<<<<<< HEAD
                timeout=10.0,
            )
            response.raise_for_status()
            return [FileInfo.model_validate(item) for item in response.json()]
=======
                timeout=10.0
            )
            response.raise_for_status()
            return response.json()
>>>>>>> 001-agentic-cad-environment-WP03

    async def read_file(self, path: str) -> str:
        """Read file contents.

        Args:
            path: Virtual file path.

        Returns:
            File content as string.
        """
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/read",
                json={"path": path},
                headers=self.headers,
<<<<<<< HEAD
                timeout=10.0,
=======
                timeout=10.0
>>>>>>> 001-agentic-cad-environment-WP03
            )
            response.raise_for_status()
            return response.json()["content"]

    async def write_file(self, path: str, content: str) -> bool:
        """Write content to a file.

        Args:
            path: Virtual file path.
            content: Content to write.

        Returns:
            True if successful.
        """
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/write",
                json={"path": path, "content": content},
                headers=self.headers,
<<<<<<< HEAD
                timeout=10.0,
=======
                timeout=10.0
>>>>>>> 001-agentic-cad-environment-WP03
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

    async def edit_file(self, path: str, edits: List[Dict[str, str]]) -> bool:
        """Edit a file with one or more operations.

        Args:
            path: Virtual file path.
            edits: List of dicts with 'old_string' and 'new_string'.

        Returns:
            True if successful.
        """
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{self.base_url}/fs/edit",
                json={"path": path, "edits": edits},
                headers=self.headers,
<<<<<<< HEAD
                timeout=10.0,
=======
                timeout=10.0
>>>>>>> 001-agentic-cad-environment-WP03
            )
            response.raise_for_status()
            return response.json()["status"] == "success"

<<<<<<< HEAD
    async def execute_python(self, code: str, timeout: int = 30) -> ExecuteResponse:
=======
    async def execute_python(self, code: str, timeout: int = 30) -> Dict[str, Any]:
>>>>>>> 001-agentic-cad-environment-WP03
        """Execute Python code in the sandboxed runtime.

        Args:
            code: Python code to execute.
            timeout: Execution timeout in seconds.

        Returns:
<<<<<<< HEAD
            ExecuteResponse model with stdout, stderr, exit_code, and timed_out.
        """
        async with httpx.AsyncClient() as client:
            # We add some buffer to the HTTP timeout to allow the runtime
=======
            Dictionary with stdout, stderr, exit_code, and timed_out.
        """
        async with httpx.AsyncClient() as client:
            # We add some buffer to the HTTP timeout to allow the runtime 
>>>>>>> 001-agentic-cad-environment-WP03
            # to finish and return the result even if it timed out internally.
            http_timeout = float(timeout) + 5.0
            response = await client.post(
                f"{self.base_url}/runtime/execute",
                json={"code": code, "timeout": timeout},
                headers=self.headers,
<<<<<<< HEAD
                timeout=http_timeout,
            )
            response.raise_for_status()
            return ExecuteResponse.model_validate(response.json())
=======
                timeout=http_timeout
            )
            response.raise_for_status()
            return response.json()
>>>>>>> 001-agentic-cad-environment-WP03

    async def get_health(self) -> Dict[str, str]:
        """Check the health of the worker service."""
        async with httpx.AsyncClient() as client:
            response = await client.get(f"{self.base_url}/health", timeout=5.0)
            response.raise_for_status()
            return response.json()
