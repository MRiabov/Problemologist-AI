from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.architect import architect_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.architect.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock(
            return_value=MagicMock(
                content="""# PLAN
Test Plan
# TODO
- [ ] Test Todo"""
            )
        )
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.architect.WorkerClient")
@patch("controller.agent.nodes.architect.RemoteFilesystemMiddleware")
async def test_architect_node_logic(mock_fs, mock_worker, mock_llm):
    # Cleanup files if they exist
    for f in ["plan.md", "todo.md"]:
        p = Path(f)
        if p.exists():
            p.unlink()

    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    result = await architect_node(state)

    # Check return value
    assert result.plan
    assert result.todo
    assert "Test Plan" in result.plan
    assert "Test Todo" in result.todo

    # Check file creation
    assert Path("plan.md").exists()
    assert Path("todo.md").exists()

    with open("plan.md") as f:
        assert f.read() == "Test Plan"

    with open("todo.md") as f:
        assert f.read() == "- [ ] Test Todo"

    # Cleanup
    Path("plan.md").unlink()
    Path("todo.md").unlink()


@pytest.mark.asyncio
@patch("controller.agent.nodes.architect.WorkerClient")
@patch("controller.agent.nodes.architect.RemoteFilesystemMiddleware")
async def test_architect_node_fallback(mock_fs, mock_worker, mock_llm):
    # Mock fallback response
    mock_llm.ainvoke.return_value = MagicMock(content="Just some text without sections")

    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    result = await architect_node(state)

    assert result.plan == "Just some text without sections"
    assert result.todo == "- [ ] Implement the plan"

    Path("plan.md").unlink()
    Path("todo.md").unlink()
