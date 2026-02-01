from langchain_core.tools import tool
import os


@tool
def write_script(content: str, path: str) -> str:
    """
    Writes content to a specific file (e.g., 'design.py', 'controller.py').
    Use this to create or overwrite scripts in the workspace.

    Args:
        content: The full content of the script.
        path: The path where the script should be saved.
    """
    try:
        # Ensure directories exist
        dirname = os.path.dirname(path)
        if dirname:
            os.makedirs(dirname, exist_ok=True)
        with open(path, "w") as f:
            f.write(content)
        return f"Successfully wrote to {path}"
    except Exception as e:
        return f"Error writing to {path}: {str(e)}"


@tool
def edit_script(find: str, replace: str, path: str) -> str:
    """
    Performs string replacement on the specified file.
    Use this to make targeted changes to existing scripts without rewriting the whole file.

    Args:
        find: The string to look for in the file.
        replace: The string to replace it with.
        path: The path of the file to edit.
    """
    try:
        if not os.path.exists(path):
            return f"Error: File {path} does not exist."
        with open(path, "r") as f:
            content = f.read()
        if find not in content:
            return (
                f"Error: '{find}' not found in {path}. Please provide an exact match."
            )
        new_content = content.replace(find, replace)
        with open(path, "w") as f:
            f.write(new_content)
        return f"Successfully edited {path}"
    except Exception as e:
        return f"Error editing {path}: {str(e)}"


@tool
def preview_design() -> str:
    """
    Runs the current design script, exports a render, and returns the view.
    Use this to visually verify your geometry before submission.
    No reward is given for this action.
    """
    # NOTE: This will be connected to the actually renderer in a future WP
    return "Preview generated. (Visual feedback would appear here in a real run)"


@tool
def submit_design(control_path: str) -> str:
    """
    Runs the current design script, performs full Workbench validation, and returns final grades.
    Uses the script at control_path for motor logic (PID control, etc).
    This call marks the end of an attempt and triggers the full simulation.

    Args:
        control_path: Path to the controller script to be used in simulation.
    """
    # NOTE: This will be connected to the MuJoCo simulation bridge in a future WP
    return "Design submitted for validation. (Simulation results would appear here)"


@tool
def search_docs(query: str) -> str:
    """
    RAG retrieval from build123d and problemologist technical documentation.
    Use this to learn about CAD library syntax, specific functions, or problem scenarios.

    Args:
        query: The search query or question to look up.
    """
    # NOTE: This will be connected to a vector store in a future WP
    return f"Search results for '{query}': (RAG snippets would appear here)"
