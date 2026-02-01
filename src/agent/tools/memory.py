import re
from datetime import datetime
from typing import List, Optional
from langchain_core.tools import tool
from src.agent.utils.workspace import Workspace

# Initialize workspace relative to execution context
# In production, this might be injected or configured via env vars
workspace = Workspace(root_dir="workspace")


@tool
def read_journal(topic: str = "") -> str:
    """
    Read the agent's journal (journal.md) to retrieve context or past memories.

    Args:
        topic: Optional topic or keyword to look for. Currently retrieves the entire journal.
    """
    content = workspace.read("journal.md")
    if not content:
        return "Journal is empty."

    if topic:
        # Split content into entries.
        # Each entry starts with "## [" (header).
        # We capture the full entry text including the header.
        entries = re.findall(r"(## \[.*?\][\s\S]*?)(?=(?:## \[|$))", content)

        filtered_entries = [
            entry for entry in entries if topic.lower() in entry.lower()
        ]

        if not filtered_entries:
            return f"No journal entries found matching topic: '{topic}'."

        return "".join(filtered_entries).strip()

    return content


@tool
def write_journal(entry: str, tags: Optional[List[str]] = None) -> str:
    """
    Write a new entry to the agent's journal (journal.md) for long-term memory.

    Args:
        entry: The text content to record.
        tags: Optional list of tags for categorization (e.g., ["design", "error"]).
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    tags_line = f"\nTags: {', '.join(tags)}" if tags else ""

    formatted_entry = f"\n\n## [{timestamp}]\n{entry}{tags_line}\n"

    workspace.append("journal.md", formatted_entry)
    return f"Journal entry recorded at {timestamp}."
