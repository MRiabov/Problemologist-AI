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

    async def get_docs_for(query: str):
        """
        Search for technical documentation and skills.
        Returns concise, actionable snippets.
        """
        from controller.agent.nodes.documentation import DocumentationNode, SharedNodeContext
        from controller.agent.config import settings
        from controller.agent.state import AgentState

        ctx = SharedNodeContext.create(
            worker_light_url=settings.spec_001_api_url, session_id=session_id
        )
        node = DocumentationNode(ctx)
        # Create a dummy state for the subagent call
        state = AgentState(session_id=session_id)
        return await node(state, query)

    async def validate_and_price(path: str = "script.py"):
        """
        Validate manufacturability and calculate costs for the assembly in a script.
        Returns a detailed report with unit costs and any DFM violations.
        """
        return await fs.validate(path)

    async def simulate(path: str = "script.py", backend: str = "GENESIS"):
        """
        Run a physics simulation to verify stability and objective completion.
        Returns simulation metrics and result summary.
        """
        from shared.simulation.schemas import SimulatorBackendType

        try:
            b_enum = SimulatorBackendType(backend.upper())
        except ValueError:
            b_enum = SimulatorBackendType.GENESIS

        return await fs.simulate(path, backend=b_enum)

    async def submit_for_review(path: str = "script.py"):
        """
        Submit the completed design for final inspection and approval.
        Requires prior validation and simulation to pass.
        """
        return await fs.submit(path)

    async def validate_costing_and_price():
        """
        Validate 'assembly_definition.yaml' and compute assembly totals.
        Autopopulates unit cost and weight fields.
        Must be called by the Planner before handoff.
        """
        return await fs.run_command(
            "python3 /skills/manufacturing-knowledge/scripts/validate_costing_and_price.py"
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
        get_docs_for,
        validate_and_price,
        simulate,
        submit_for_review,
        validate_costing_and_price,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
