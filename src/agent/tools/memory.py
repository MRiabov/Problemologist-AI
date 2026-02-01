import difflib
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
        topic: Optional topic or keyword to filter entries by.
    """
    content = workspace.read("journal.md")
    if not content:
        return "Journal is empty."

    if topic:
        # Split content into entries.
        # Each entry starts with "## [" (header).
        # We capture the full entry text including the header.
        entries = re.findall(r"(## \[.*?\][\s\S]*?)(?=(?:## \[|$))", content)

        def is_match(entry_text: str, search_topic: str) -> bool:
            entry_lower = entry_text.lower()
            topic_lower = search_topic.lower()

            # 1. Exact substring match
            if topic_lower in entry_lower:
                return True

            # 2. Fuzzy match
            # Split into tokens to match words
            entry_words = re.findall(r"\w+", entry_lower)
            topic_words = re.findall(r"\w+", topic_lower)

            if not topic_words:
                return False

            # Check if every word in topic has a close match in entry
            for t_word in topic_words:
                matches = difflib.get_close_matches(
                    t_word, entry_words, n=1, cutoff=0.7
                )
                if not matches:
                    return False
            return True

        filtered_entries = [entry for entry in entries if is_match(entry, topic)]

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
