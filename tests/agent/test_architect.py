from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.planner import planner_node
from controller.agent.state import AgentState
from controller.agent.nodes.base import SharedNodeContext


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.base.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock(
            return_value=AIMessage(
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
@patch("controller.agent.nodes.planner.record_worker_events")
@patch("controller.agent.nodes.planner.create_react_agent")
@patch("controller.agent.nodes.planner.SharedNodeContext")
async def test_architect_node_logic(
    mock_ctx_cls, mock_agent_factory, mock_record_events, mock_llm
):
    # Create a real SharedNodeContext but with mocked attributes to satisfy beartype
    # We use MagicMock for complex objects but ensure they return what's expected
    mock_pm = MagicMock()
    mock_pm.render.return_value = "Rendered prompt"

    mock_fs = MagicMock()

    async def mock_read_file(f):
        return {
            "plan.md": "## 1. Solution Overview\nTest Overview\n## 2. Parts List\n- Part A\n## 3. Assembly Strategy\n1. Step 1\n## 4. Cost & Weight Budget\n- $10\n## 5. Risk Assessment\n- Risk 1",
            "todo.md": "- [ ] Test Todo",
        }.get(f, "")

    mock_fs.read_file = AsyncMock(side_effect=mock_read_file)
    mock_fs.write_file = AsyncMock(return_value=True)

    mock_ctx = SharedNodeContext(
        worker_url="http://worker",
        session_id="test-session",
        pm=mock_pm,
        llm=mock_llm,
        worker_client=MagicMock(),
        fs=mock_fs,
    )
    # Mock the get_callbacks method which is NOT a dataclass field
    mock_ctx.get_callbacks = MagicMock(return_value=[])

    mock_ctx_cls.create.return_value = mock_ctx

    # Mock agent instance
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [AIMessage(content="Finished planning.")]
    }
    mock_agent_factory.return_value = mock_agent

    state = AgentState(task="Build a robot", session_id="test-session")

    result = await planner_node(state)

    # Check return value
    assert result.plan
    assert result.todo
    assert "Test Overview" in result.plan
    assert "Test Todo" in result.todo


@pytest.mark.asyncio
@patch("controller.agent.nodes.planner.record_worker_events")
@patch("controller.agent.nodes.planner.create_react_agent")
@patch("controller.agent.nodes.planner.SharedNodeContext")
async def test_architect_node_fallback(
    mock_ctx_cls, mock_agent_factory, mock_record_events, mock_llm
):
    mock_pm = MagicMock()
    mock_pm.render.return_value = "Rendered prompt"

    mock_fs = MagicMock()
    mock_fs.read_file = AsyncMock(return_value="")
    mock_fs.write_file = AsyncMock(return_value=True)

    mock_ctx = SharedNodeContext(
        worker_url="http://worker",
        session_id="test-session",
        pm=mock_pm,
        llm=mock_llm,
        worker_client=MagicMock(),
        fs=mock_fs,
    )
    mock_ctx.get_callbacks = MagicMock(return_value=[])

    mock_ctx_cls.create.return_value = mock_ctx

    # Mock agent instance that fails validation (by returning no files)
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [AIMessage(content="Just some text without writing files")]
    }
    mock_agent_factory.return_value = mock_agent

    state = AgentState(task="Build a robot", session_id="test-session")

    result = await planner_node(state)

    # Should be rejected because validation fails (after retries)
    from controller.agent.state import AgentStatus

    assert result.status == AgentStatus.PLAN_REJECTED
    assert "Planner output validation failed" in result.feedback
