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
        Validate pricing YAML, autopopulate fields, and output price.
        Runs the manufacturing knowledge validation script.
        """
        script_path = "/skills/manufacturing-knowledge/scripts/validate_costing_and_price.py"
        return await fs.run_command(f"python3 {script_path}")

    async def submit_for_review():
        """
        Submits the whole assembly for a review to the Reviewer agent node.
        Note: Requires script.py to have been successfully validated and simulated.
        """
        return await fs.run_command("python3 -c 'from utils import submit_for_review; from shared.workers.loader import load_component_from_script; submit_for_review(load_component_from_script(\"script.py\"))'")

    async def refuse_plan(reason: str):
        """
        Refuse the current plan with a specific reason.
        Use this when the planner's instructions are physically impossible or over-budget.
        """
        import shlex

        safe_reason = shlex.quote(reason)
        return await fs.run_command(
            f"python3 -c 'from utils import refuse_plan; refuse_plan({safe_reason})'"
        )

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
        submit_for_review,
        refuse_plan,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
