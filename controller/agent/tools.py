from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import RunCommandToolEvent
from shared.simulation.schemas import SimulatorBackendType


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

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        search_cots_catalog,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now includes high-level engineering tools (simulate, preview, etc.).
    """
    common_tools = get_common_tools(fs, session_id)

    async def simulate(
        script_path: str, backend: SimulatorBackendType = SimulatorBackendType.GENESIS
    ) -> dict:
        """
        Run physics simulation for the design.
        Genesis is used by default for high-fidelity (fluids, FEM).
        MuJoCo can be selected for fast rigid-body only runs.
        """
        return await fs.simulate(script_path, backend=backend)

    async def preview_design(
        script_path: str, pitch: float = -45.0, yaw: float = 45.0
    ) -> dict:
        """
        Generate a 3D preview image of the current CAD design.
        Returns the path to the generated image.
        """
        return await fs.preview(script_path, pitch=pitch, yaw=yaw)

    async def validate_costing_and_price() -> dict:
        """
        Validate 'assembly_definition.yaml' and autopopulate pricing/weight totals.
        Returns the validation result and calculated totals.
        """
        res = await fs.validate_costing_and_price()
        return {
            "success": res.exit_code == 0,
            "stdout": res.stdout,
            "stderr": res.stderr,
        }

    async def get_docs_for(type_name: str) -> str:
        """
        Retrieve documentation and usage examples for a build123d entity or skill.
        Example: get_docs_for("Box") or get_docs_for("fastener_hole")
        """
        return await fs.get_docs_for(type_name)

    async def submit_for_review(script_path: str = "script.py") -> dict:
        """
        Submit the current design and documentation for reviewer approval.
        This tool should be called once the task in TODO is complete.
        """
        return await fs.submit(script_path)

    return [
        *common_tools,
        simulate,
        preview_design,
        validate_costing_and_price,
        get_docs_for,
        submit_for_review,
    ]
