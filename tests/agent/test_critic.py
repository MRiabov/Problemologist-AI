from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from shared.enums import ReviewDecision
from controller.agent.nodes.plan_reviewer import ReviewResult, plan_reviewer_node
from controller.agent.state import AgentState, AgentStatus


@pytest.fixture
def mock_llm():
    with patch("dspy.LM") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        # Mock structured output for the parser
        instance.with_structured_output = MagicMock(return_value=instance)
        yield instance


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock()
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.plan_reviewer.dspy.ReAct")
async def test_critic_node_approve(mock_react_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(decision=ReviewDecision.APPROVED, reason="Looks good.")
    )
    mock_react_cls.return_value = mock_program

    state = AgentState(
        task="Build a part", journal="Implementation details", session_id="test-session"
    )

    result = await plan_reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Looks good" in result.feedback
    assert "Plan Critic Decision: approved" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.plan_reviewer.dspy.ReAct")
async def test_critic_node_reject(mock_react_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(
            decision=ReviewDecision.REJECT_PLAN, reason="Simulation failed."
        )
    )
    mock_react_cls.return_value = mock_program

    state = AgentState(task="Build a part", journal="", session_id="test-session")

    result = await plan_reviewer_node(state)

    assert result.status == AgentStatus.PLAN_REJECTED
    assert "Simulation failed" in result.feedback
    assert "Plan Critic Decision: reject_plan" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.plan_reviewer.dspy.ReAct")
async def test_critic_node_no_artifacts(mock_react_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(
            decision=ReviewDecision.APPROVED, reason="No artifacts but journal is fine."
        )
    )
    mock_react_cls.return_value = mock_program

    state = AgentState(task="Build a part", journal="", session_id="test-session")

    result = await plan_reviewer_node(state)

    # Even without artifacts, LLM decides based on journal
    assert result.status == AgentStatus.APPROVED
    mock_program.assert_called_once()
