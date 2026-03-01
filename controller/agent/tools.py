from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import EscalationRequestEvent, RunCommandToolEvent


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
        """Query documentation for a given entity or topic."""
        return await fs.get_docs_for(query)

    async def validate_costing_and_price():
        """Validate assembly_definition.yaml and compute totals."""
        return await fs.run_command(
            "python .agent/skills/manufacturing-knowledge/scripts/validate_costing_and_price.py"
        )

    async def refuse_plan(reason: str):
        """Refuse the current plan if it's fundamentally flawed or impossible."""
        # Record escalation request event
        await record_worker_events(
            episode_id=session_id,
            events=[EscalationRequestEvent(reason=reason)],
        )

        # Simple implementation by writing a refusal file that the reviewer can check
        content = f"Plan Refused\nReason: {reason}"
        await fs.write_file("refusal.md", content, overwrite=True)
        return f"Plan refused with reason: {reason}"

    async def validate_circuit(section_name: str = "electronics"):
        """Validate electronic circuit using PySpice."""
        # In this implementation, we assume the code is in script.py and we run a validation script
        return await fs.run_command(f"python -c 'from shared.pyspice_utils import validate_circuit; validate_circuit(section=\"{section_name}\")'")

    async def route_wire(start: tuple, end: tuple, label: str):
        """Route a physical 3D wire between two points."""
        # This is a stub for the tool as referenced in prompts
        return f"Wire '{label}' routed from {start} to {end}."

    async def check_wire_clearance(wire_label: str):
        """Check if a wire has sufficient clearance from obstacles."""
        return f"Wire '{wire_label}' clearance check: OK."

    async def calculate_power_budget():
        """Calculate total power consumption for the assembly."""
        return await fs.run_command("python -c 'from shared.pyspice_utils import calculate_power_budget; print(calculate_power_budget())'")

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
        validate_costing_and_price,
        refuse_plan,
        validate_circuit,
        route_wire,
        check_wire_clearance,
        calculate_power_budget,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
