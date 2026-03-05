from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult
from shared.observability.schemas import RunCommandToolEvent


def _tool_name(tool: Callable) -> str:
    return getattr(tool, "name", getattr(tool, "__name__", str(tool)))


def filter_tools_for_agent(
    fs: RemoteFilesystemMiddleware, tools: list[Callable]
) -> list[Callable]:
    """Filter tool list using per-agent allowlist from config/agents_config.yaml."""
    allowed = fs.policy.get_allowed_tools(fs.agent_role)
    if allowed is None:
        return tools
    return [tool for tool in tools if _tool_name(tool) in allowed]


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

    tools = [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        search_cots_catalog,
    ]
    return filter_tools_for_agent(fs, tools)


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Uses the common toolset only.
    """
    return get_common_tools(fs, session_id)


def get_engineer_planner_tools(
    fs: RemoteFilesystemMiddleware,
    session_id: str,
    planner_node_type: AgentName = AgentName.ENGINEER_PLANNER,
) -> list[Callable]:
    """
    Planner-specific toolset for engineer/electronics planners.

    Includes explicit `submit_plan()` so planner completion is an intentional action.
    """
    common_tools = get_common_tools(fs, session_id)

    async def submit_plan() -> dict:
        """
        Validate planner artifacts and explicitly submit the planning handoff.

        Returns:
            {"ok": bool, "status": "submitted"|"rejected", "errors": [...], "node_type": "..."}
        """
        from worker_heavy.utils.file_validation import validate_node_output

        # Engineer planner and electronics planner share the same planner artifacts.
        required_files = ["plan.md", "todo.md", "assembly_definition.yaml"]
        artifacts: dict[str, str] = {}
        missing_files: list[str] = []

        for rel_path in required_files:
            if not await fs.exists(rel_path):
                missing_files.append(rel_path)
                continue
            content = await fs.read_file(rel_path)
            if not content.strip():
                missing_files.append(rel_path)
                continue
            artifacts[rel_path] = content

        if missing_files:
            result = PlannerSubmissionResult(
                ok=False,
                status="rejected",
                errors=[f"Missing required file: {p}" for p in missing_files],
                node_type=planner_node_type,
            )
            return result.model_dump(mode="json")

        is_valid, errors = validate_node_output(AgentName.ENGINEER_PLANNER, artifacts)
        result = PlannerSubmissionResult(
            ok=is_valid,
            status="submitted" if is_valid else "rejected",
            errors=errors,
            node_type=planner_node_type,
        )
        return result.model_dump(mode="json")

    return filter_tools_for_agent(fs, [*common_tools, submit_plan])
