import asyncio
import json
from typing import Any

from langchain_core.tools import tool

from src.environment.runtime import ToolRuntime

# Runtimes management
_DEFAULT_RUNTIME_ID: str = "default"
_RUNTIMES: dict[str, ToolRuntime] = {}
_FALLBACK_RUNTIME: ToolRuntime | None = None


def _get_fallback_runtime() -> ToolRuntime:
    global _FALLBACK_RUNTIME
    if _FALLBACK_RUNTIME is None:
        _FALLBACK_RUNTIME = ToolRuntime("workspace")
    return _FALLBACK_RUNTIME


def register_runtime(runtime_id: str, runtime: ToolRuntime):
    """Registers a runtime instance."""
    global _RUNTIMES
    _RUNTIMES[runtime_id] = runtime


def get_runtime(runtime_id: str | None = None) -> ToolRuntime:
    """
    Gets a registered runtime by ID.

    Lookup order:
    1. Explicit runtime_id if provided and registered
    2. Default runtime if registered
    3. Lazy-created fallback runtime (for standalone usage)
    """
    if runtime_id and runtime_id in _RUNTIMES:
        return _RUNTIMES[runtime_id]

    if _DEFAULT_RUNTIME_ID in _RUNTIMES:
        return _RUNTIMES[_DEFAULT_RUNTIME_ID]

    return _get_fallback_runtime()


async def _execute_tool(
    tool_name: str, tool_runtime: ToolRuntime | None, **kwargs
) -> Any:
    """Dispatches tool call to runtime."""
    rt = tool_runtime or get_runtime()
    # Pass agent_role if we can extract it from the context?
    # For now, we assume dispatch handles it if needed.
    return await asyncio.to_thread(rt.dispatch, tool_name, kwargs)


async def start_session_async(session_id: str = "vlm-cad-session") -> str:
    """
    Starts a persistent sandbox session asynchronously.

    Args:
        session_id: Identifier for the session (default: vlm-cad-session).

    Returns:
        Status message indicating success or failure.
    """
    rt = get_runtime()
    return await asyncio.to_thread(rt.start_session, session_id)


# --- Agent Tools ---


@tool
async def write_file(
    content: str, path: str, mode: str = "overwrite", tool_runtime: Any | None = None
) -> str:
    """
    Writes content to a specific file or appends to it.
    Args:
        content: The text content to write.
        path: The path where the file should be saved.
        mode: Either 'overwrite' or 'append' (default: 'overwrite').
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool(
        "write_file", tool_runtime, content=content, path=path, mode=mode
    )


@tool
async def edit_file(
    path: str, find: str, replace: str, tool_runtime: Any | None = None
) -> str:
    """
    Performs string replacement on the specified file.
    Args:
        path: The path of the file to edit.
        find: The exact string to look for.
        replace: The string to replace it with.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool(
        "edit_file", tool_runtime, path=path, find=find, replace=replace
    )


@tool
async def preview_design(
    path: str = "design.py", tool_runtime: Any | None = None
) -> str:
    """
    Runs the current design script, exports a render, and returns the view.
    Args:
        path: Path to the script to preview (default: design.py).
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("preview_design", tool_runtime, filename=path)


@tool
async def submit_design(control_path: str, tool_runtime: Any | None = None) -> str:
    """
    Runs the current design script, performs full Workbench validation, and returns final grades.
    Args:
        control_path: Path to the controller script to be used in simulation.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("submit_design", tool_runtime, control_path=control_path)


@tool
async def search_docs(query: str, tool_runtime: Any | None = None) -> str:
    """
    RAG retrieval from build123d and problemologist technical documentation.
    Args:
        query: The search query or question to look up.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("search_docs", tool_runtime, query=query)


@tool
async def check_manufacturability(
    design_file: str = "design.py",
    process: str = "cnc",
    quantity: int = 1,
    tool_runtime: Any | None = None,
) -> dict:
    """
    Provides a detailed DFM (Design for Manufacturing) report and pricing breakdown.
    Use this to identify manufacturability violations (draft, undercuts, thickness)
    and to understand cost drivers (e.g. cooling time for IM, stock volume for CNC).
    Args:
        design_file: The name of the script file to analyze.
        process: The manufacturing process ('cnc' or 'injection_molding').
        quantity: Target production quantity.
        tool_runtime: Injected runtime (do not provide).
    """
    output = await _execute_tool(
        "check_manufacturability",
        tool_runtime,
        design_file=design_file,
        process=process,
        quantity=quantity,
    )
    if isinstance(output, str):
        try:
            return json.loads(output)
        except Exception:
            return {"error": output}
    return output


@tool
async def view_file(path: str, tool_runtime: Any | None = None) -> str:
    """
    Reads the content of any file in the workspace or documentation.
    Args:
        path: Relative path to the file.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("view_file", tool_runtime, path=path)


@tool
async def run_command(command: str, tool_runtime: Any | None = None) -> str:
    """
    Executes a shell command inside the persistent sandbox environment.
    Args:
        command: The shell command to execute.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("run_command", tool_runtime, command=command)


@tool
async def lint_script(
    filename: str = "design.py", tool_runtime: Any | None = None
) -> str:
    """
    Runs static analysis (Ruff, Pyrefly) on a script.
    Args:
        filename: The name of the script to lint.
        tool_runtime: Injected runtime (do not provide).
    """
    return await _execute_tool("lint_script", tool_runtime, filename=filename)


# --- Skill Tools ---


@tool
async def read_skill(
    skill_name: str,
    filename: str = "SKILL.md",
    resource_type: str | None = None,
    tool_runtime: Any | None = None,
) -> str:
    """Reads the content of a specialized skill."""
    return await _execute_tool(
        "read_skill",
        tool_runtime,
        skill_name=skill_name,
        filename=filename,
        resource_type=resource_type,
    )


@tool
async def list_skills(tool_runtime: Any | None = None) -> str:
    """Lists all available specialized skills."""
    return await _execute_tool("list_skills", tool_runtime)


@tool
async def list_skill_files(skill_name: str, tool_runtime: Any | None = None) -> str:
    """Lists all files within a specialized skill folder."""
    return await _execute_tool("list_skill_files", tool_runtime, skill_name=skill_name)


@tool
async def init_skill(skill_name: str, tool_runtime: Any | None = None) -> str:
    """Initializes a new skill directory."""
    return await _execute_tool("init_skill", tool_runtime, skill_name=skill_name)


@tool
async def package_skill(skill_name: str, tool_runtime: Any | None = None) -> str:
    """Validates and packages a skill."""
    return await _execute_tool("package_skill", tool_runtime, skill_name=skill_name)


@tool
async def update_skill(
    skill_name: str,
    content: str,
    filename: str = "SKILL.md",
    resource_type: str | None = None,
    tool_runtime: Any | None = None,
) -> str:
    """Updates or adds information to a specialized skill folder."""
    return await _execute_tool(
        "update_skill",
        tool_runtime,
        skill_name=skill_name,
        content=content,
        filename=filename,
        resource_type=resource_type,
    )


@tool
async def run_skill_script(
    skill_name: str,
    script_name: str,
    arguments: str = "",
    tool_runtime: Any | None = None,
) -> str:
    """Executes a specialized script located within a skill."""
    return await _execute_tool(
        "run_skill_script",
        tool_runtime,
        skill_name=skill_name,
        script_name=script_name,
        arguments=arguments,
    )


# --- COTS Tools ---


@tool
async def search_parts(query: str, tool_runtime: Any | None = None) -> str:
    """Search for COTS parts by name or ID."""
    return await _execute_tool("search_parts", tool_runtime, query=query)


@tool
async def preview_part(part_id: str, tool_runtime: Any | None = None) -> str:
    """Get visual preview and details for a specific COTS part ID."""
    return await _execute_tool("preview_part", tool_runtime, part_id=part_id)
