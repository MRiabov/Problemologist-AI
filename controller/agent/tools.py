from typing import List, Optional
from langchain_core.tools import tool
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.cots.agent import search_cots_catalog


def get_engineer_tools(fs: RemoteFilesystemMiddleware):
    @tool
    async def list_files(path: str = "/") -> List[dict]:
        """List files in the workspace."""
        return await fs.list_files(path)

    @tool
    async def read_file(path: str) -> str:
        """Read a file's content."""
        return await fs.read_file(path)

    @tool
    async def write_file(path: str, content: str) -> bool:
        """Write content to a file."""
        return await fs.write_file(path, content)

    @tool
    async def grep(
        pattern: str, path: Optional[str] = None, glob: Optional[str] = None
    ) -> List[dict]:
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    return [list_files, read_file, write_file, grep, search_cots_catalog]
