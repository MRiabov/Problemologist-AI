"""
Single source of truth for agent tools.

All consumers (graph.py, actor.py, etc.) should import from here
to avoid tool list quadruplication.
"""

from src.agent.tools.env_adapter import (
    check_manufacturability,
    edit_file,
    init_skill,
    lint_script,
    list_skill_files,
    list_skills,
    package_skill,
    preview_design,
    preview_part,
    read_skill,
    run_command,
    run_skill_script,
    search_docs,
    search_parts,
    submit_design,
    update_skill,
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
    read_skill,
    list_skills,
    list_skill_files,
    init_skill,
    package_skill,
    update_skill,
    run_skill_script,
]
