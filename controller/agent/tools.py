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

    async def list_files(path: str = "/"):
        """List files in the workspace (filesystem)."""
        return await fs.list_files(path)

    async def read_file(path: str):
        """Read a file's content from the workspace."""
        return await fs.read_file(path)

    async def write_file(path: str, content: str, overwrite: bool = False):
        """Write content to a file in the workspace."""
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit_file(path: str, old_string: str, new_string: str):
        """Edit a file by replacing old_string with new_string."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep(pattern: str, path: str | None = None, glob: str | None = None):
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    async def execute_command(command: str):
        """Execute a shell command in the workspace."""
        # Record the command execution event
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=command)],
        )
        return await fs.run_command(command)

    async def inspect_topology(target_id: str, script_path: str = "script.py") -> dict:
        """
        Inspect geometric properties of a selected feature (face, edge, part).
        Returns center, normal, area, and bounding box.
        """
        return await fs.inspect_topology(target_id, script_path)

    async def cots_search(query: str) -> str:
        """
        Search for COTS parts in the catalog.
        Wraps the search_cots_catalog function to provide a simpler interface for the agent.
        """
        return search_cots_catalog(query)

    async def validate_costing_and_price() -> str:
        """
        Validate the assembly definition and pricing.
        Runs the validate_and_price.py script to check cost and weight constraints.
        """
        cmd = "python3 skills/manufacturing-knowledge/scripts/validate_and_price.py"
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=cmd)],
        )
        return await fs.run_command(cmd)

    async def get_docs_for(query: str) -> str:
        """
        Search for documentation for a specific term or concept.
        Searches skills and build123d docs.
        """
        # Simple sanitization
        safe_query = query.replace('"', '\\"').replace("'", "\\'")
        cmd = f"python3 -c 'from worker_light.utils.docs import get_docs_for; print(get_docs_for(\"{safe_query}\"))'"

        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=cmd)],
        )
        return await fs.run_command(cmd)

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        cots_search,
        validate_costing_and_price,
        get_docs_for,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
