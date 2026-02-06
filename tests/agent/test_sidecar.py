import shutil
import pytest
import sys
from unittest.mock import MagicMock, patch, AsyncMock

# MOCK BEFORE IMPORTING APPLICATION CODE
# The langchain imports hang in this environment, so we mock them globally.
# We must mock specific modules that are imported by sidecar.py
mock_openai = MagicMock()
sys.modules["langchain_openai"] = mock_openai

mock_core = MagicMock()
mock_messages = MagicMock()
mock_core.messages = mock_messages
sys.modules["langchain_core"] = mock_core
sys.modules["langchain_core.messages"] = mock_messages

# Now safe to import
from pathlib import Path
from src.agent.nodes.sidecar import SidecarNode, sidecar_node
from src.agent.state import AgentState


@pytest.fixture
def mock_llm():
    # Patch the class where it is defined since we use lazy import
    with patch("langchain_openai.ChatOpenAI") as mock:
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


@pytest.mark.asyncio
async def test_sidecar_logic_suggest_skill(mock_llm):
    """Test the core logic of SidecarNode directly (blocking)."""
    test_dir = Path("test_suggested_skills")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(
        task="Test task", journal="I struggled with Box until I imported it correctly."
    )

    # Directly await the node logic
    await node(state)

    # Verify file creation
    expected_file = test_dir / "build123d_import_trick.md"
    assert expected_file.exists()

    content = expected_file.read_text()
    assert "Build123D Import Trick" in content
    assert "from build123d import Box" in content

    # Cleanup
    if test_dir.exists():
        shutil.rmtree(test_dir)


@pytest.mark.asyncio
async def test_sidecar_logic_no_skill(mock_llm):
    """Test SidecarNode logic when no skill is suggested."""
    mock_llm.ainvoke.return_value = MagicMock(content="No new skills identified.")

    test_dir = Path("test_suggested_skills_empty")
    if test_dir.exists():
        shutil.rmtree(test_dir)

    node = SidecarNode(suggested_skills_dir=str(test_dir))
    state = AgentState(task="Easy task", journal="Everything worked perfectly.")

    await node(state)

    # Verify no file created
    assert not any(test_dir.iterdir()) if test_dir.exists() else True

    if test_dir.exists():
        shutil.rmtree(test_dir)


@pytest.mark.asyncio
async def test_sidecar_node_background_execution(mock_llm):
    """Test that sidecar_node returns immediately and launches background task."""
    state = AgentState(task="Bg task", journal="Log")

    with patch("src.agent.nodes.sidecar.asyncio.create_task") as mock_create_task:
        result = await sidecar_node(state)

        # Should return empty dict immediately
        assert result == {}

        # Should have launched a task
        mock_create_task.assert_called_once()
