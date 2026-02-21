from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.execution_reviewer import execution_reviewer_node
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
@patch("controller.agent.nodes.execution_reviewer.ExecutionReviewerNode._run_program")
async def test_execution_reviewer_node_approve(mock_run, mock_worker):
    # Mock _run_program return value
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(decision=ReviewDecision.APPROVED, reason="Looks good.")
        ),
        [],
        "\nExecution Reviewer Journal",
    )

    state = AgentState(
        task="Build a part", journal="Implementation details", session_id="test-session"
    )

    result = await execution_reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Looks good" in result.feedback
    assert "Critic Decision: approved" in result.journal
    assert any(
        isinstance(m, AIMessage) and "Review decision: approved" in m.content
        for m in result.messages
    )


@pytest.mark.asyncio
@patch("controller.agent.nodes.execution_reviewer.ExecutionReviewerNode._run_program")
async def test_execution_reviewer_node_reject(mock_run, mock_worker):
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.REJECT_CODE, reason="Simulation failed."
            )
        ),
        [],
        "\nExecution Reviewer Journal",
    )

    state = AgentState(task="Build a part", journal="", session_id="test-session")

    result = await execution_reviewer_node(state)

    assert result.status == AgentStatus.CODE_REJECTED
    assert "Simulation failed" in result.feedback
    assert "Critic Decision: reject_code" in result.journal
