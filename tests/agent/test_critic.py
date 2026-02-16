import json
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.reviewer import ReviewerNode
from controller.agent.state import AgentState, AgentStatus


@pytest.fixture
def mock_agent():
    with patch("controller.agent.nodes.base.ChatOpenAI"), \
         patch("controller.agent.nodes.reviewer.create_react_agent") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = {
            "messages": [
                AIMessage(
                    content="""---
decision: approve
---
Looks good."""
                )
            ]
        }
        yield instance


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock()
        yield instance


@pytest.mark.asyncio
async def test_critic_node_approve(mock_agent, mock_worker):
    # Mock reports
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "success", "results": "passed"}),  # sim report
        "Part is manufacturable.",  # mfg report
    ]

    from controller.agent.nodes.base import SharedNodeContext
    ctx = SharedNodeContext.create("http://worker:8001", "default-session")
    node = ReviewerNode(context=ctx)
    state = AgentState(task="Build a part", journal="Implementation details")

    result = await node(state)

    assert result.status == AgentStatus.APPROVED
    assert "Looks good" in result.feedback
    assert "Critic Decision: APPROVE" in result.journal


@pytest.mark.asyncio
async def test_critic_node_reject(mock_agent, mock_worker):
    mock_agent.ainvoke.return_value = {
        "messages": [
            AIMessage(
                content="""---
decision: reject_code
---
Simulation failed."""
            )
        ]
    }
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "error", "message": "collision"}),
        "Too expensive.",
    ]

    from controller.agent.nodes.base import SharedNodeContext
    ctx = SharedNodeContext.create("http://worker:8001", "default-session")
    node = ReviewerNode(context=ctx)
    state = AgentState(task="Build a part", journal="")

    result = await node(state)

    assert result.status == AgentStatus.CODE_REJECTED
    assert "Simulation failed" in result.feedback
    assert "Critic Decision: REJECT_CODE" in result.journal


@pytest.mark.asyncio
async def test_critic_node_no_artifacts(mock_agent, mock_worker):
    mock_worker.read_file.side_effect = Exception("File not found")

    from controller.agent.nodes.base import SharedNodeContext
    ctx = SharedNodeContext.create("http://worker:8001", "default-session")
    node = ReviewerNode(context=ctx)
    state = AgentState(task="Build a part", journal="")

    result = await node(state)

    # Even without artifacts, LLM decides based on journal
    assert result.status
    mock_agent.ainvoke.assert_called_once()
