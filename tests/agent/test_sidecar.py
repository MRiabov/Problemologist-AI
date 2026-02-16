import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.skills import SkillsNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_agent_call():
    with patch("controller.agent.nodes.skills.ChatOpenAI"), \
         patch("controller.agent.nodes.skills.create_react_agent") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = {
            "messages": [
                AIMessage(
                    content="",
                    tool_calls=[
                        {
                            "name": "save_suggested_skill",
                            "args": {
                                "title": "build123d_import_trick",
                                "content": """# Build123D Import Trick
## Problem
Syntax error when using Box.
## Solution
Import Box from build123d.
## Example
```python
from build123d import Box
```""",
                            },
                            "id": "call_1",
                            "type": "tool_call",
                        }
                    ],
                )
            ]
        }
        yield instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill(mock_agent_call):
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    # Mock git repo to avoid actual git operations
    with patch("controller.agent.nodes.skills.Repo"), \
         patch("controller.agent.nodes.skills.WorkerClient"), \
         patch("controller.agent.nodes.skills.RemoteFilesystemMiddleware"):
        node = SkillsNode(suggested_skills_dir=str(test_dir))
        state = AgentState(
            task="Test task",
            journal="I struggled with Box until I imported it correctly.",
        )

        result = await node(state)

    assert "Identified and recorded 1 new skills" in result.journal

    # Cleanup
    shutil.rmtree(test_dir, ignore_errors=True)


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_no_skill(mock_agent_call):
    mock_agent_call.ainvoke.return_value = {
        "messages": [AIMessage(content="No new skills identified.")]
    }

    test_dir = Path("test_suggested_skills")
    # Mock git repo
    with patch("controller.agent.nodes.skills.Repo"), \
         patch("controller.agent.nodes.skills.WorkerClient"), \
         patch("controller.agent.nodes.skills.RemoteFilesystemMiddleware"):
        node = SkillsNode(suggested_skills_dir=str(test_dir))
        state = AgentState(task="Easy task", journal="Everything worked perfectly.")

        result = await node(state)

    assert "No new skills identified" in result.journal

    shutil.rmtree(test_dir, ignore_errors=True)
