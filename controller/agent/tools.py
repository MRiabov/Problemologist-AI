from collections.abc import Callable

from langchain_core.tools import tool

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import RunCommandToolEvent


def get_common_tools(fs: RemoteFilesystemMiddleware, session_id: str) -> list[Callable]:
    """
    Get the set of common tools available to all agents (Engineer, Benchmark, etc.).
    Includes filesystem operations and COTS catalog search.
    """

    @tool
    async def list_files(path: str = "/"):
        """List files in the workspace (filesystem)."""
        return await fs.list_files(path)

    @tool
    async def read_file(path: str):
        """Read a file's content from the workspace."""
        return await fs.read_file(path)

    @tool
    async def write_file(path: str, content: str, overwrite: bool = False):
        """Write content to a file in the workspace."""
        return await fs.write_file(path, content, overwrite=overwrite)

    @tool
    async def edit_file(path: str, old_string: str, new_string: str):
        """Edit a file by replacing old_string with new_string."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    @tool
    async def grep(pattern: str, path: str | None = None, glob: str | None = None):
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    @tool
    async def execute_command(command: str):
        """Execute a shell command in the workspace."""
        # Record the command execution event
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=command)],
        )
        return await fs.run_command(command)

    @tool
    async def inspect_topology(target_id: str, script_path: str = "script.py") -> dict:
        """
        Inspect geometric properties of a selected feature (face, edge, part).
        Returns center, normal, area, and bounding box.
        """
        return await fs.inspect_topology(target_id, script_path)

    @tool
    async def simulate(script_path: str = "script.py"):
        """Trigger physics simulation and return stability report."""
        return await fs.simulate(script_path)

    @tool
    async def verify(script_path: str = "script.py", num_runs: int = 5):
        """Trigger multi-run physics verification with position jitter."""
        return await fs.verify(script_path, num_runs=num_runs)

    @tool
    async def validate(script_path: str = "script.py"):
        """Trigger geometric validation check."""
        return await fs.validate(script_path)

    @tool
    async def analyze(
        script_path: str = "script.py", method: str = "cnc", quantity: int = 1
    ):
        """Trigger manufacturing analysis for cost and manufacturability."""
        return await fs.analyze(script_path, method=method, quantity=quantity)

    @tool
    async def preview(script_path: str = "script.py"):
        """Generate a 3D render preview of the design."""
        return await fs.preview(script_path)

    @tool
    async def submit(script_path: str = "script.py"):
        """Handover the current design for review."""
        return await fs.submit(script_path)

    @tool
    async def validate_circuit():
        """Validate the electronics circuit defined in assembly_definition.yaml."""
        import yaml

        if not await fs.exists("assembly_definition.yaml"):
            return "Error: assembly_definition.yaml not found."

        content = await fs.read_file("assembly_definition.yaml")
        data = yaml.safe_load(content)
        if "electronics" not in data or not data["electronics"]:
            return "Error: No electronics section found in assembly_definition.yaml"

        return await fs.validate_circuit(data["electronics"])

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        search_cots_catalog,
        simulate,
        verify,
        validate,
        analyze,
        preview,
        submit,
        validate_circuit,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
