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
@patch("controller.agent.nodes.reviewer.create_react_agent")
async def test_critic_node_approve(mock_agent_factory, mock_llm, mock_worker):
    # Mock agent
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [AIMessage(content="--- \ndecision: approved\n--- \nLooks good.")]
    }
    mock_agent_factory.return_value = mock_agent

    # Mock structured parser return value
    mock_llm.ainvoke.return_value = ReviewResult(
        decision=CriticDecision.APPROVE, reason="Looks good."
    )

    # Mock reports
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "success", "results": "passed"}),  # sim report
        "Part is manufacturable.",  # mfg report
    ]

    state = AgentState(task="Build a part", journal="Implementation details")

    result = await reviewer_node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Looks good" in result.feedback
    assert "Critic Decision: APPROVE" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.reviewer.create_react_agent")
async def test_critic_node_reject(mock_agent_factory, mock_llm, mock_worker):
    # Mock agent
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [
            AIMessage(content="--- \ndecision: rejected\n--- \nSimulation failed.")
        ]
    }
    mock_agent_factory.return_value = mock_agent

    # Mock structured parser return value
    mock_llm.ainvoke.return_value = ReviewResult(
        decision=CriticDecision.REJECT_CODE, reason="Simulation failed."
    )

    mock_worker.read_file.side_effect = [
        json.dumps({"status": "error", "message": "collision"}),
        "Too expensive.",
    ]

    state = AgentState(task="Build a part", journal="")

    result = await reviewer_node(state)

    assert result.status == AgentStatus.CODE_REJECTED
    assert "Simulation failed" in result.feedback
    assert "Critic Decision: REJECT_CODE" in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.reviewer.create_react_agent")
async def test_critic_node_no_artifacts(mock_agent_factory, mock_llm, mock_worker):
    # Mock agent
    mock_agent = AsyncMock()
    mock_agent.ainvoke.return_value = {
        "messages": [
            AIMessage(
                content="--- \ndecision: approved\n--- \nNo artifacts but journal is fine."
            )
        ]
    }
    mock_agent_factory.return_value = mock_agent

    # Mock structured parser return value
    mock_llm.ainvoke.return_value = ReviewResult(
        decision=CriticDecision.APPROVE, reason="No artifacts but journal is fine."
    )

    mock_worker.read_file.side_effect = Exception("File not found")

    state = AgentState(task="Build a part", journal="")

    result = await reviewer_node(state)

    # Even without artifacts, LLM decides based on journal
    assert result.status
    mock_agent.ainvoke.assert_called_once()
