from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import RunCommandToolEvent


def get_common_tools(fs: RemoteFilesystemMiddleware, session_id: str) -> list[Callable]:
    """
    Get the set of common tools available to all agents (Engineer, Benchmark, etc.).
    Includes filesystem operations and COTS catalog search.
    Aligned with architectural names in desired_architecture.md.
    """

    async def ls_info(path: str = "/"):
        """List files in the workspace with metadata."""
        return await fs.list_files(path)

    async def read(path: str):
        """Read a file's content from the workspace."""
        return await fs.read_file(path)

    async def write(path: str, content: str, overwrite: bool = False):
        """Write content to a file in the workspace."""
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit(path: str, old_string: str, new_string: str):
        """Edit a file by replacing old_string with new_string."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep_raw(pattern: str, path: str | None = None, glob: str | None = None):
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    async def execute(command: str):
        """Execute a shell command in the workspace."""
        # Record the command execution event
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=command)],
        )
        return await fs.run_command(command)

    async def glob_info(pattern: str, path: str | None = None):
        """List files matching a glob pattern."""
        return await fs.glob_info(pattern, path)

    async def upload_files(files: dict[str, str]):
        """Upload multiple files to the workspace."""
        return await fs.upload_files(files)

    async def download_files(paths: list[str]):
        """Download multiple files from the workspace."""
        return await fs.download_files(paths)

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
        execute,
        glob_info,
        upload_files,
        download_files,
        inspect_topology,
        search_cots_catalog,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent (Mechanical/Electrical).
    Includes common tools plus specialized simulation and DFM tools.
    """
    tools = get_common_tools(fs, session_id)

    async def validate_and_price(script_path: str = "script.py"):
        """Validate the design for manufacturability and calculate costs."""
        return await fs.validate(script_path)

    async def simulate(script_path: str = "script.py"):
        """Run physics simulation to verify the design."""
        return await fs.simulate(script_path)

    async def submit_for_review(script_path: str = "script.py"):
        """Submit the design for reviewer approval."""
        return await fs.submit(script_path)

    async def preview_design(script_path: str = "script.py", pitch: float = -45.0, yaw: float = 45.0):
        """Render a preview image of the current CAD design."""
        return await fs.preview(script_path, pitch=pitch, yaw=yaw)

    async def get_docs_for(topic: str):
        """Get documentation for a specific CAD or engineering topic."""
        # Simple implementation: grep through skills and documentation
        return await fs.grep(topic, path="skills")

    tools.extend([
        validate_and_price,
        simulate,
        submit_for_review,
        preview_design,
        get_docs_for
    ])
    return tools


def get_planner_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Get tools for Planner agents."""
    tools = get_common_tools(fs, session_id)

    async def validate_costing_and_price():
        """Validate the assembly_definition.yaml and autopopulate costs."""
        return await fs.run_command("python3 skills/manufacturing-knowledge/scripts/validate_and_price.py")

    tools.append(validate_costing_and_price)
    return tools


def get_benchmark_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Get tools for Benchmark Generator agents."""
    tools = get_common_tools(fs, session_id)

    async def validate(script_path: str = "script.py"):
        """Validate benchmark geometry for intersections and bounds."""
        return await fs.validate(script_path)

    async def simulate(script_path: str = "script.py"):
        """Run benchmark simulation to verify it is solvable and valid."""
        return await fs.simulate(script_path)

    async def submit_for_review(script_path: str = "script.py"):
        """Submit the benchmark for reviewer approval."""
        return await fs.submit(script_path)

    async def get_docs_for(topic: str):
        """Get documentation for CAD/benchmarking topics."""
        return await fs.grep(topic, path="skills")

    tools.extend([
        validate,
        simulate,
        submit_for_review,
        get_docs_for
    ])
    return tools
