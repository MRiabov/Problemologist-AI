from typing import Optional

from langchain_core.tools import tool

from src.agent.tools.env_adapter import (
    edit_script_async,
    init_skill_async,
    list_skill_files_async,
    list_skills_async,
    package_skill_async,
    preview_design_async,
    preview_part_async,
    read_skill_async,
    read_script_async,
    search_docs_async,
    search_parts_async,
    submit_design_async,
    update_skill_async,
    write_script_async,
    check_manufacturability_async,
)


@tool
async def write_script(content: str, path: str) -> str:
    """
    Writes content to a specific file (e.g., 'design.py', 'controller.py').
    Use this to create or overwrite scripts in the workspace.

    Args:
        content: The full content of the script.
        path: The path where the script should be saved.
    """
    return await write_script_async(content, path)


@tool
async def read_script(path: str = "design.py") -> str:
    """
    Reads the content of a script from the workspace.
    Use this to inspect your existing code before editing or refactoring.

    Args:
        path: The path of the file to read (default: 'design.py').
    """
    return await read_script_async(path)


@tool
async def edit_script(find: str, replace: str, path: str) -> str:
    """
    Performs string replacement on the specified file.
    Use this to make targeted changes to existing scripts without rewriting the whole file.

    Args:
        find: The string to look for in the file.
        replace: The string to replace it with.
        path: The path of the file to edit.
    """
    return await edit_script_async(path, find, replace)


@tool
async def preview_design(path: str = "design.py") -> str:
    """
    Runs the current design script, exports a render, and returns the view.
    Use this to visually verify your geometry before submission.
    No reward is given for this action.

    Args:
        path: Path to the script to preview (default: design.py).
    """
    return await preview_design_async(path)


@tool
async def submit_design(control_path: str) -> str:
    """
    Runs the current design script, performs full Workbench validation, and returns final grades.
    Uses the script at control_path for motor logic (PID control, etc).
    This call marks the end of an attempt and triggers the full simulation.

    Args:
        control_path: Path to the controller script to be used in simulation.
    """
    return await submit_design_async(control_path)


@tool
async def search_docs(query: str) -> str:
    """
    RAG retrieval from build123d and problemologist technical documentation.
    Use this to learn about CAD library syntax, specific functions, or problem scenarios.

    Args:
        query: The search query or question to look up.
    """
    return await search_docs_async(query)


@tool
async def update_skill(
    skill_name: str,
    content: str,
    filename: str = "SKILL.md",
    resource_type: Optional[str] = None,
) -> str:
    """
    Updates or adds information to a specialized skill folder (e.g., 'build123d_cad_drafting_skill').
    Use this to capture new knowledge, patterns, or documentation discovered during task execution.

    Args:
        skill_name: The name of the skill to update.
        content: The Markdown or Python content to write.
        filename: The filename (e.g., 'SKILL.md', 'patterns.md', 'helper.py').
        resource_type: Mandatory if filename is not 'SKILL.md'. One of: 'scripts', 'references', 'assets'.
    """
    return await update_skill_async(skill_name, content, filename, resource_type)


@tool
async def read_skill(
    skill_name: str, filename: str = "SKILL.md", resource_type: Optional[str] = None
) -> str:
    """
    Reads the content of a specialized skill.
    MANDATORY: You must read the 'build123d_cad_drafting_skill' before writing any build123d code.

    Args:
        skill_name: The name of the skill to read.
        filename: The filename (e.g., 'SKILL.md').
        resource_type: Optional. One of: 'scripts', 'references', 'assets'.
    """
    return await read_skill_async(skill_name, filename, resource_type)


@tool
async def list_skills() -> str:
    """
    Lists all available specialized skills.
    Use this to discover what knowledge categories are available.
    """
    return await list_skills_async()


@tool
async def list_skill_files(skill_name: str) -> str:
    """
    Lists all files within a specialized skill folder, grouped by type.
    Use this to see the available reference documents or scripts for a skill.

    Args:
        skill_name: The name of the skill to inspect.
    """
    return await list_skill_files_async(skill_name)


@tool
async def init_skill(skill_name: str) -> str:
    """
    Initializes a new skill directory with the canonical structure (SKILL.md, scripts/, references/, assets/).
    Use this when you want to create a new category of specialized knowledge.

    Args:
        skill_name: The name of the skill to create (hyphen-case, e.g., 'my-new-skill').
    """
    return await init_skill_async(skill_name)


@tool
async def package_skill(skill_name: str) -> str:
    """
    Validates and packages a skill into a distributable .skill file.
    Use this when you have finished developing or updating a skill.

    Args:
        skill_name: The name of the skill to package.
    """
    return await package_skill_async(skill_name)


@tool
async def run_skill_script(
    skill_name: str, script_name: str, arguments: str = ""
) -> str:
    """
    Executes a specialized script located within a skill's 'scripts' folder.
    Use this to perform deterministic tasks or fetch dynamic data defined by a skill.

    Args:
        skill_name: The name of the skill containing the script.
        script_name: The filename of the script (e.g., 'fetch_data.py').
        arguments: Optional string of arguments to pass to the script.
    """
    return await run_skill_script_async(skill_name, script_name, arguments)


@tool
async def search_parts(query: str) -> str:
    """
    Search for COTS parts by name or ID. Returns a list of matches.
    Use this to find standard components like motors, bearings, or fasteners.

    Args:
        query: The search query or part name to look up.
    """
    return await search_parts_async(query)


@tool
async def preview_part(part_id: str) -> str:
    """
    Get visual preview and details for a specific COTS part ID.
    Returns a description and a Python recipe to instantiate it.

    Args:
        part_id: The namespaces ID of the part (e.g., 'bd_warehouse:motor:Nema17').
    """
    return await preview_part_async(part_id)


@tool
async def check_manufacturability(
    design_file: str = "design.py", process: str = "cnc", quantity: int = 1
) -> dict:
    """
    Checks if the design in the specified file can be manufactured using the target process.
    Supported processes: 'cnc', 'injection_molding'.

    Args:
        design_file: The name of the script file to analyze.
        process: The manufacturing process to check against ('cnc' or 'injection_molding').
        quantity: Target production quantity (affects cost).
    """
    return await check_manufacturability_async(design_file, process, quantity)