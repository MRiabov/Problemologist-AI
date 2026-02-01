from langchain_core.tools import tool

from src.agent.utils.workspace import Workspace

# Initialize workspace
workspace = Workspace(root_dir="workspace")


@tool
def update_plan(status: str, notes: str) -> str:
    """
    Update the current session's plan (plan.md).
    Use this to record progress, mark steps as complete, or revise the strategy.

    Args:
        status: The current status of the plan (e.g., "In Progress", "Completed",
                "Failed", "Replanning").
        notes: detailed notes or the updated plan content.
    """
    content = f"## Status: {status}\n\n{notes}\n"
    # We overwrite the plan file to keep it current, or we could append.
    # The spec says "Update the current session's scratchpad".
    # Overwriting seems appropriate for a "current state" of the plan.
    workspace.write("plan.md", content)
    return f"Plan updated with status: {status}"
