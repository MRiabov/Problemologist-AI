import pytest
from src.agent.tools.registry import AGENT_TOOLS

def test_skill_tools_are_registered():
    """
    Verifies that all skill management tools are included in the AGENT_TOOLS registry.
    """
    registered_tool_names = [tool.name for tool in AGENT_TOOLS]

    missing_tools = []
    required_tools = [
        "read_skill",
        "list_skills",
        "list_skill_files",
        "init_skill",
        "package_skill",
        "update_skill",
        "run_skill_script"
    ]

    for tool_name in required_tools:
        if tool_name not in registered_tool_names:
            missing_tools.append(tool_name)

    assert not missing_tools, f"The following tools are missing from AGENT_TOOLS: {missing_tools}"
