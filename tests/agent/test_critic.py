import json
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.critic import CriticNode
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.critic.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="""---
decision: approve
---
Looks good."""
        )
        yield instance


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.critic.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock()
        yield instance


@pytest.mark.asyncio
async def test_critic_node_approve(mock_llm, mock_worker):
    # Mock reports
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "success", "results": "passed"}),  # sim report
        "Part is manufacturable.",  # mfg report
    ]

    node = CriticNode()
    state = AgentState(task="Build a part", journal="Implementation details")

    result = await node(state)

    assert result.status == "approved"
    assert "Looks good" in result.feedback
    assert "Critic Decision: APPROVE" in result.journal


@pytest.mark.asyncio
async def test_critic_node_reject(mock_llm, mock_worker):
    mock_llm.ainvoke.return_value = MagicMock(
        content="""---
decision: reject_code
---
Simulation failed."""
    )
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "error", "message": "collision"}),
        "Too expensive.",
    ]

    node = CriticNode()
    state = AgentState(task="Build a part", journal="")

    result = await node(state)

    assert result.status == "code_rejected"
    assert "Simulation failed" in result.feedback
    assert "Critic Decision: REJECT" in result.journal


@pytest.mark.asyncio
async def test_critic_node_no_artifacts(mock_llm, mock_worker):
    mock_worker.read_file.side_effect = Exception("File not found")

    node = CriticNode()
    state = AgentState(task="Build a part", journal="")

    result = await node(state)

    # Even without artifacts, LLM decides based on journal
    assert result.status
    mock_llm.ainvoke.assert_called_once()
