import json
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.reviewer import reviewer_node, ReviewResult, CriticDecision
from controller.agent.state import AgentState, AgentStatus


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.base.ChatOpenAI") as mock:
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
@patch("controller.agent.nodes.reviewer.dspy.CodeAct")
async def test_critic_node_approve(mock_codeact_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(decision=CriticDecision.APPROVE, reason="Looks good.")
    )
    mock_codeact_cls.return_value = mock_program

    state = AgentState(task="Build a part", journal="Implementation details", session_id="test-session")

    result = await reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Looks good" in result.feedback
    assert "Critic Decision: APPROVE" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.reviewer.dspy.CodeAct")
async def test_critic_node_reject(mock_codeact_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(decision=CriticDecision.REJECT_CODE, reason="Simulation failed.")
    )
    mock_codeact_cls.return_value = mock_program

    state = AgentState(task="Build a part", journal="", session_id="test-session")

    result = await reviewer_node(state)

    assert result.status == AgentStatus.CODE_REJECTED
    assert "Simulation failed" in result.feedback
    assert "Critic Decision: REJECT_CODE" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.reviewer.dspy.CodeAct")
async def test_critic_node_no_artifacts(mock_codeact_cls, mock_llm, mock_worker):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(
        review=ReviewResult(decision=CriticDecision.APPROVE, reason="No artifacts but journal is fine.")
    )
    mock_codeact_cls.return_value = mock_program

    state = AgentState(task="Build a part", journal="", session_id="test-session")

    result = await reviewer_node(state)

    # Even without artifacts, LLM decides based on journal
    assert result.status == AgentStatus.APPROVED
    mock_program.assert_called_once()
