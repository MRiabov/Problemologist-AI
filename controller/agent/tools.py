from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import RunCommandToolEvent


def get_common_tools(fs: RemoteFilesystemMiddleware, session_id: str) -> list[Callable]:
    """
    Get the set of common tools available to all agents (Engineer, Benchmark, etc.).
    Includes filesystem operations and COTS catalog search.
    """

    async def ls_info(path: str = "/"):
        """
        Structured listing with file metadata using os.scandir.
        Returns a list of FileInfo dicts with name, path, is_dir, and size.
        """
        return await fs.list_files(path)

    async def read(path: str):
        """Read file content with line numbers using a single shell command."""
        return await fs.read_file(path)

    async def write(path: str, content: str, overwrite: bool = False):
        """Create a new file. Returns WriteResult; error populated on failure."""
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit(path: str, old_string: str, new_string: str):
        """Edit a file by replacing string occurrences. Returns EditResult."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep_raw(pattern: str, path: str | None = None, glob: str | None = None):
        """Structured search results or error string for invalid input."""
        return await fs.grep(pattern, path, glob)

    async def glob_info(pattern: str):
        """Structured glob matching returning a list of path strings."""
        # Implementation fallback using grep with empty pattern if no native glob endpoint
        matches = await fs.grep("", pattern=None, glob=pattern)
        # Convert GrepMatch to something resembling FileInfo or just list unique paths
        paths = sorted(list({m.path for m in matches}))
        return paths

    async def execute(command: str):
        """Execute a command in the sandbox and return ExecuteResponse."""
        # Record the command execution event
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=command)],
        )
        return await fs.run_command(command)

    async def upload_files(files: dict[str, str]):
        """Upload multiple files to the sandbox. Input is a mapping of path to content."""
        results = []
        for path, content in files.items():
            res = await fs.write_file(path, content, overwrite=True)
            results.append({"path": path, "success": res})
        return results

    async def download_files(paths: list[str]):
        """Download multiple files from the sandbox."""
        results = {}
        for path in paths:
            try:
                content = await fs.read_file(path)
                results[path] = content
            except Exception as e:
                results[path] = f"Error: {e}"
        return results

    async def inspect_topology(target_id: str, script_path: str = "script.py") -> dict:
        """
        Inspect geometric properties of a selected feature (face, edge, part).
        Returns center, normal, area, and bounding box.
        """
        return await fs.inspect_topology(target_id, script_path)

    return [
        ls_info,
        read,
        write,
        edit,
        grep_raw,
        glob_info,
        execute,
        upload_files,
        download_files,
        inspect_topology,
        search_cots_catalog,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
