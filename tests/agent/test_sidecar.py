import shutil
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.sidecar import SidecarNode
from controller.agent.state import AgentState
from shared.type_checking import type_check


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.sidecar.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="""SKILL: Build123D Import Trick
CONTENT:
# Build123D Import Trick
## Problem
Syntax error when using Box.
## Solution
Import Box from build123d.
## Example
```python
from build123d import Box
```"""
        )
        yield instance


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_suggest_skill(mock_llm):
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(
        task="Test task", journal="I struggled with Box until I imported it correctly."
    )

    result = await node(state)

    assert "Suggested skill build123d_import_trick" in result.journal
    skill_file = test_dir / "build123d_import_trick.md"
    assert skill_file.exists()

    with skill_file.open("r") as f:
        content = f.read()
        assert "Build123D Import Trick" in content
        assert "from build123d import Box" in content

    # Cleanup
    shutil.rmtree(test_dir)


@type_check
@pytest.mark.asyncio
async def test_sidecar_node_no_skill(mock_llm):
    mock_llm.ainvoke.return_value = MagicMock(content="No new skills identified.")

    test_dir = Path("test_suggested_skills")
    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(task="Easy task", journal="Everything worked perfectly.")

    result = await node(state)

    assert "No new skills identified" in result.journal

    if test_dir.exists():
        shutil.rmtree(test_dir)
