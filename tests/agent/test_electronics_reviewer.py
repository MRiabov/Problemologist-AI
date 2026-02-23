from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.electronics_reviewer import electronics_reviewer_node
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
@patch("controller.agent.nodes.electronics_reviewer.ElectronicsReviewerNode._run_program")
async def test_electronics_reviewer_node_approve(mock_run, mock_worker):
    # Mock _run_program return value
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(decision=ReviewDecision.APPROVED, reason="Circuit is correct.")
        ),
        [],
        "\nElectronics Reviewer Journal",
    )

    state = AgentState(
        task="Design a circuit", journal="Electronics details", session_id="test-session"
    )

    result = await electronics_reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Circuit is correct" in result.feedback
    assert "Electronics Review Decision: approved" in result.journal
    assert any(
        isinstance(m, AIMessage) and "Electronics Review decision: approved" in m.content
        for m in result.messages
    )


@pytest.mark.asyncio
@patch("controller.agent.nodes.electronics_reviewer.ElectronicsReviewerNode._run_program")
async def test_electronics_reviewer_node_reject(mock_run, mock_worker):
    mock_run.return_value = (
        MagicMock(
            review=ReviewResult(
                decision=ReviewDecision.REJECT_CODE, reason="Short circuit detected."
            )
        ),
        [],
        "\nElectronics Reviewer Journal",
    )

    state = AgentState(task="Design a circuit", journal="", session_id="test-session")

    result = await electronics_reviewer_node(state)

    assert result.status == AgentStatus.CODE_REJECTED
    assert "Short circuit" in result.feedback
    assert "Electronics Review Decision: reject_code" in result.journal
