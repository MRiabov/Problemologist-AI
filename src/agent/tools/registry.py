"""
Single source of truth for agent tools.

All consumers (graph.py, actor.py, etc.) should import from here
to avoid tool list quadruplication.
"""

from src.agent.tools.env_adapter import (
    check_manufacturability,
    edit_file,
    lint_script,
    preview_design,
    preview_part,
    run_command,
    search_docs,
    search_parts,
    submit_design,
    view_file,
    write_file,
)
from src.agent.tools.memory import read_journal

# Canonical list of all agent tools
AGENT_TOOLS = [
    write_file,
    edit_file,
    view_file,
    run_command,
    preview_design,
    submit_design,
    search_docs,
    check_manufacturability,
    read_journal,
    search_parts,
    preview_part,
    lint_script,
]

# Metadata for Graph Routing
preview_design.triggers_review = True
submit_design.triggers_review = True
