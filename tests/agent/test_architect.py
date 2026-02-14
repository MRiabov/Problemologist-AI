from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.planner import planner_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.planner.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock(
            return_value=MagicMock(
                content="""# PLAN
## 1. Solution Overview
Test Overview
## 2. Parts List
- Part A
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- $10
## 5. Risk Assessment
- Risk 1
# TODO
- [ ] Test Todo"""
            )
        )
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.planner.WorkerClient")
@patch("controller.agent.nodes.planner.RemoteFilesystemMiddleware")
async def test_architect_node_logic(mock_fs, mock_worker, mock_llm):
    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    result = await planner_node(state)

    # Check return value
    assert result.plan
    assert result.todo
    assert "Test Overview" in result.plan
    assert "Test Todo" in result.todo

    # Check mock file creation
    fs_instance.write_file.assert_any_call(
        "plan.md",
        mock_llm.ainvoke.return_value.content.split("# TODO")[0]
        .replace("# PLAN", "")
        .strip(),
    )
    fs_instance.write_file.assert_any_call("todo.md", "- [ ] Test Todo")


@pytest.mark.asyncio
@patch("controller.agent.nodes.planner.WorkerClient")
@patch("controller.agent.nodes.planner.RemoteFilesystemMiddleware")
async def test_architect_node_fallback(mock_fs, mock_worker, mock_llm):
    # Mock fallback response (missing sections)
    mock_llm.ainvoke.return_value = MagicMock(content="Just some text without sections")

    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    result = await planner_node(state)

    # Should be rejected because validation fails
    from controller.agent.state import AgentStatus

    assert result.status == AgentStatus.PLAN_REJECTED
    assert "Planner output validation failed" in result.feedback

    # Check that it didn't write files (or we don't care, but usually it shouldn't proceed)
    fs_instance.write_file.assert_not_called()
