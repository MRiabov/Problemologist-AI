from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.plan_reviewer import plan_reviewer_node
from controller.agent.state import AgentState, AgentStatus
from shared.enums import ReviewDecision
from shared.models.schemas import ReviewResult


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="# Mock content")
        instance.exists = AsyncMock(return_value=True)
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.plan_reviewer.PlanReviewerNode._run_program")
async def test_plan_reviewer_node_approve(mock_run, mock_worker):
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.APPROVED, reason="Plan looks solid."
            )
        ),
        [],
        "\nPlan Reviewer Journal",
    )

    state = AgentState(
        task="Design a part", journal="Planning details", session_id="test-session"
    )

    result = await plan_reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Plan looks solid" in result.feedback
    assert "Plan Critic Decision: approved" in result.journal
    assert any(
        isinstance(m, AIMessage) and "Plan Review decision: approved" in m.content
        for m in result.messages
    )


@pytest.mark.asyncio
@patch("controller.agent.nodes.plan_reviewer.PlanReviewerNode._run_program")
async def test_plan_reviewer_node_reject(mock_run, mock_worker):
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.REJECT_PLAN, reason="Budget exceeded."
            )
        ),
        [],
        "\nPlan Reviewer Journal",
    )

    state = AgentState(task="Design a part", journal="", session_id="test-session")

    result = await plan_reviewer_node(state)

    assert result.status == AgentStatus.PLAN_REJECTED
    assert "Budget exceeded" in result.feedback
    assert "Plan Critic Decision: reject_plan" in result.journal
