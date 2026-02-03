from typing import Optional, Any

from langchain_core.tools import tool

from src.agent.tools.env_adapter import (
    write_file_async,
    edit_file_async,
    preview_design_async,
    preview_part_async,
    search_docs_async,
    search_parts_async,
    submit_design_async,
    check_manufacturability_async,
    view_file_async,
    run_command_async,
    start_session_async,
    stop_session_async,
    run_skill_script_async,
    lint_script_async,
    read_skill_async,
    list_skills_async,
    list_skill_files_async,
    init_skill_async,
    package_skill_async,
    update_skill_async,
)


@tool
async def write_file(content: str, path: str, mode: str = "overwrite", tool_runtime: Optional[Any] = None) -> str:
    """
    Writes content to a specific file or appends to it.
    Use this to create scripts, update logs, or record journal entries.
    If path is 'journal.md', automatic timestamping is applied in append mode.

    Args:
        content: The text content to write.
        path: The path where the file should be saved (e.g., 'design.py', 'journal.md').
        mode: Either 'overwrite' or 'append' (default: 'overwrite').
        tool_runtime: Injected runtime (do not provide).
    """
    return await write_file_async(content, path, mode, tool_runtime=tool_runtime)


@tool
async def edit_file(path: str, find: str, replace: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Performs string replacement on the specified file.
    Use this to make targeted changes to existing files without rewriting them.

    Args:
        path: The path of the file to edit.
        find: The exact string to look for.
        replace: The string to replace it with.
        tool_runtime: Injected runtime (do not provide).
    """
    return await edit_file_async(path, find, replace, tool_runtime=tool_runtime)


@tool
async def preview_design(path: str = "design.py", tool_runtime: Optional[Any] = None) -> str:
    """
    Runs the current design script, exports a render, and returns the view.
    Use this to visually verify your geometry before submission.
    No reward is given for this action.

    Args:
        path: Path to the script to preview (default: design.py).
        tool_runtime: Injected runtime (do not provide).
    """
    return await preview_design_async(path, tool_runtime=tool_runtime)


@tool
async def submit_design(control_path: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Runs the current design script, performs full Workbench validation, and returns final grades.
    Uses the script at control_path for motor logic (PID control, etc).
    This call marks the end of an attempt and triggers the full simulation.

    Args:
        control_path: Path to the controller script to be used in simulation.
        tool_runtime: Injected runtime (do not provide).
    """
    return await submit_design_async(control_path, tool_runtime=tool_runtime)


@tool
async def search_docs(query: str, tool_runtime: Optional[Any] = None) -> str:
    """
    RAG retrieval from build123d and problemologist technical documentation.
    Use this to learn about CAD library syntax, specific functions, or problem scenarios.

    Args:
        query: The search query or question to look up.
        tool_runtime: Injected runtime (do not provide).
    """
    return await search_docs_async(query, tool_runtime=tool_runtime)


@tool
async def update_skill(
    skill_name: str,
    content: str,
    filename: str = "SKILL.md",
    resource_type: Optional[str] = None,
    tool_runtime: Optional[Any] = None,
) -> str:
    """
    Updates or adds information to a specialized skill folder (e.g., 'build123d_cad_drafting_skill').
    Use this to capture new knowledge, patterns, or documentation discovered during task execution.

    Args:
        skill_name: The name of the skill to update.
        content: The Markdown or Python content to write.
        filename: The filename (e.g., 'SKILL.md', 'patterns.md', 'helper.py').
        resource_type: Mandatory if filename is not 'SKILL.md'. One of: 'scripts', 'references', 'assets'.
        tool_runtime: Injected runtime (do not provide).
    """
    return await update_skill_async(skill_name, content, filename, resource_type, tool_runtime=tool_runtime)


@tool
async def read_skill(
    skill_name: str, filename: str = "SKILL.md", resource_type: Optional[str] = None, tool_runtime: Optional[Any] = None
) -> str:
    """
    Reads the content of a specialized skill.
    MANDATORY: You must read the 'build123d_cad_drafting_skill' before writing any build123d code.

    Args:
        skill_name: The name of the skill to read.
        filename: The filename (e.g., 'SKILL.md').
        resource_type: Optional. One of: 'scripts', 'references', 'assets'.
        tool_runtime: Injected runtime (do not provide).
    """
    return await read_skill_async(skill_name, filename, resource_type, tool_runtime=tool_runtime)


@tool
async def list_skills(tool_runtime: Optional[Any] = None) -> str:
    """
    Lists all available specialized skills.
    Use this to discover what knowledge categories are available.

    Args:
        tool_runtime: Injected runtime (do not provide).
    """
    return await list_skills_async(tool_runtime=tool_runtime)


@tool
async def list_skill_files(skill_name: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Lists all files within a specialized skill folder, grouped by type.
    Use this to see the available reference documents or scripts for a skill.

    Args:
        skill_name: The name of the skill to inspect.
        tool_runtime: Injected runtime (do not provide).
    """
    return await list_skill_files_async(skill_name, tool_runtime=tool_runtime)


@tool
async def init_skill(skill_name: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Initializes a new skill directory with the canonical structure (SKILL.md, scripts/, references/, assets/).
    Use this when you want to create a new category of specialized knowledge.

    Args:
        skill_name: The name of the skill to create (hyphen-case, e.g., 'my-new-skill').
        tool_runtime: Injected runtime (do not provide).
    """
    return await init_skill_async(skill_name, tool_runtime=tool_runtime)


@tool
async def package_skill(skill_name: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Validates and packages a skill into a distributable .skill file.
    Use this when you have finished developing or updating a skill.

    Args:
        skill_name: The name of the skill to package.
        tool_runtime: Injected runtime (do not provide).
    """
    return await package_skill_async(skill_name, tool_runtime=tool_runtime)


@tool
async def run_skill_script(
    skill_name: str, script_name: str, arguments: str = "", tool_runtime: Optional[Any] = None
) -> str:
    """
    Executes a specialized script located within a skill's 'scripts' folder.
    Use this to perform deterministic tasks or fetch dynamic data defined by a skill.

    Args:
        skill_name: The name of the skill containing the script.
        script_name: The filename of the script (e.g., 'fetch_data.py').
        arguments: Optional string of arguments to pass to the script.
        tool_runtime: Injected runtime (do not provide).
    """
    return await run_skill_script_async(skill_name, script_name, arguments, tool_runtime=tool_runtime)


@tool
async def search_parts(query: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Search for COTS parts by name or ID. Returns a list of matches.
    Use this to find standard components like motors, bearings, or fasteners.

    Args:
        query: The search query or part name to look up.
        tool_runtime: Injected runtime (do not provide).
    """
    return await search_parts_async(query, tool_runtime=tool_runtime)


@tool
async def preview_part(part_id: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Get visual preview and details for a specific COTS part ID.
    Returns a description and a Python recipe to instantiate it.

    Args:
        part_id: The namespaces ID of the part (e.g., 'bd_warehouse:motor:Nema17').
        tool_runtime: Injected runtime (do not provide).
    """
    return await preview_part_async(part_id, tool_runtime=tool_runtime)


@tool
async def check_manufacturability(
    design_file: str = "design.py", process: str = "cnc", quantity: int = 1, tool_runtime: Optional[Any] = None
) -> dict:
    """
    Checks if the design in the specified file can be manufactured using the target process.
    Supported processes: 'cnc', 'injection_molding'.

    Args:
        design_file: The name of the script file to analyze.
        process: The manufacturing process to check against ('cnc' or 'injection_molding').
        quantity: Target production quantity (affects cost).
        tool_runtime: Injected runtime (do not provide).
    """
    return await check_manufacturability_async(design_file, process, quantity, tool_runtime=tool_runtime)


@tool
async def view_file(path: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Reads the content of any file in the workspace or documentation.
    Use this to inspect scripts, skill files, or configuration.

    Args:
        path: Relative path to the file (e.g., 'design.py', 'docs/skills/build123d_cad_drafting_skill/SKILL.md').
        tool_runtime: Injected runtime (do not provide).
    """
    return await view_file_async(path, tool_runtime=tool_runtime)


@tool
async def run_command(command: str, tool_runtime: Optional[Any] = None) -> str:
    """
    Executes a shell command inside the persistent sandbox environment.
    Use this to run your Python scripts (`python design.py`), list files (`ls`), or check syntax.
    You MUST have an active session for this to be effective.

    Args:
        command: The shell command to execute.
        tool_runtime: Injected runtime (do not provide).
    """
    return await run_command_async(command, tool_runtime=tool_runtime)


async def start_session(session_id: str, tool_runtime: Optional[Any] = None) -> str:
    """Starts the sandbox session for the agent."""
    return await start_session_async(session_id, tool_runtime=tool_runtime)


async def stop_session(tool_runtime: Optional[Any] = None) -> str:
    """Stops the sandbox session for the agent."""
    return await stop_session_async(tool_runtime=tool_runtime)


@tool
async def lint_script(filename: str = "design.py", tool_runtime: Optional[Any] = None) -> str:
    """
    Runs static analysis (Ruff, Pyrefly) on a script in the workspace.
    Use this to find syntax errors or potential bugs before running or submitting.

    Args:
        filename: The name of the script to lint (default: 'design.py').
        tool_runtime: Injected runtime (do not provide).
    """
    return await lint_script_async(filename, tool_runtime=tool_runtime)
