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

    async def validate_costing_and_price():
        """
        Validates assembly_definition.yaml and autopopulates objectives.yaml
        with calculated unit cost and weight totals.
        """
        res = await fs.run_command(
            "python3 skills/manufacturing-knowledge/scripts/validate_costing_and_price.py"
        )
        return {
            "success": res.exit_code == 0,
            "stdout": res.stdout,
            "stderr": res.stderr,
        }

    async def simulate(script_path: str = "script.py"):
        """
        Triggers a full physics simulation of the assembly.
        """
        return await fs.simulate(script_path)

    async def preview_design(script_path: str = "script.py"):
        """
        Generates a 3D preview image of the current design.
        """
        return await fs.preview(script_path)

    async def submit_for_review(script_path: str = "script.py"):
        """
        Submits the current design and plan for engineering review.
        """
        return await fs.submit(script_path)

    async def get_docs_for(query: str):
        """
        Searches the engineering documentation for the specified query.
        """
        # Sanitization: escape quotes to mitigate injection
        safe_query = query.replace("'", "\\'").replace('"', '\\"')
        res = await fs.run_command(f"python3 scripts/search_docs.py '{safe_query}'")
        return res.stdout

    async def validate_design(script_path: str = "script.py"):
        """
        Performs geometric and manufacturability validation of the design.
        """
        return await fs.validate(script_path)

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        search_cots_catalog,
        validate_costing_and_price,
        simulate,
        preview_design,
        submit_for_review,
        get_docs_for,
        validate_design,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
