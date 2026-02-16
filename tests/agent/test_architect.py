from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.planner import planner_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_agent():
    with patch("controller.agent.nodes.base.ChatOpenAI"), \
         patch("controller.agent.nodes.planner.create_react_agent") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock(
            return_value={
                "messages": [
                    AIMessage(
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
                ]
            }
        )
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.base.WorkerClient")
@patch("controller.agent.nodes.base.RemoteFilesystemMiddleware")
async def test_architect_node_logic(mock_fs, mock_worker, mock_agent):
    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    full_plan = """## 1. Solution Overview
Test Overview
## 2. Parts List
- Part A
## 3. Assembly Strategy
1. Step 1
## 4. Cost & Weight Budget
- $10
## 5. Risk Assessment
- Risk 1"""

    fs_instance.read_file = AsyncMock(side_effect=[
        full_plan, "- [ ] Test Todo", # Attempt 1
        full_plan, "- [ ] Test Todo", # Attempt 2
        full_plan, "- [ ] Test Todo"  # Attempt 3
    ])

    result = await planner_node(state)

    # Check return value
    assert result.plan
    assert result.todo
    assert "Test Overview" in result.plan
    assert "Test Todo" in result.todo


@pytest.mark.asyncio
@patch("controller.agent.nodes.base.WorkerClient")
@patch("controller.agent.nodes.base.RemoteFilesystemMiddleware")
async def test_architect_node_fallback(mock_fs, mock_worker, mock_agent):
    # Mock fallback response (missing sections)
    mock_agent.ainvoke.return_value = {
        "messages": [AIMessage(content="Just some text without sections")]
    }

    state = AgentState(task="Build a robot")

    # Configure mock_fs instance to support async context manager or async methods
    fs_instance = mock_fs.return_value
    fs_instance.write_file = AsyncMock()

    result = await planner_node(state)

    # Should be rejected because validation fails
    from controller.agent.state import AgentStatus

    assert result.status == AgentStatus.PLAN_REJECTED
    assert "Planner output validation failed" in result.feedback
