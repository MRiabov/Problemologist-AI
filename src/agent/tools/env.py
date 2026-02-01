from langchain_core.tools import tool
from src.agent.tools.env_adapter import (
    write_script_async,
    edit_script_async,
    preview_design_async,
    submit_design_async,
    search_docs_async,
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
