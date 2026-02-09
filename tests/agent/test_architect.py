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

    # Check mock file creation
    fs_instance.write_file.assert_any_call("plan.md", "Test Plan")
    fs_instance.write_file.assert_any_call("todo.md", "- [ ] Test Todo")


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

    # Check mock file creation
    fs_instance.write_file.assert_any_call("plan.md", "Just some text without sections")
    fs_instance.write_file.assert_any_call("todo.md", "- [ ] Implement the plan")
