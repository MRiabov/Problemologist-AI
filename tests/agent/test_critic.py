import json
import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from src.agent.nodes.critic import CriticNode
from src.agent.state import AgentState

@pytest.fixture
def mock_llm():
    with patch("src.agent.nodes.critic.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(content="""DECISION: APPROVE
FEEDBACK: Looks good.""")
        yield instance

@pytest.fixture
def mock_worker():
    with patch("src.agent.nodes.critic.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock()
        yield instance

@pytest.mark.asyncio
async def test_critic_node_approve(mock_llm, mock_worker):
    # Mock reports
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "success", "results": "passed"}), # sim report
        "Part is manufacturable." # mfg report
    ]
    
    node = CriticNode()
    state = AgentState(task="Build a part", journal="Implementation details")
    
    result = await node(state)
    
    assert result["status"] == "approved"
    assert "Looks good" in result["feedback"]
    assert "Critic Decision: APPROVE" in result["journal"]

@pytest.mark.asyncio
async def test_critic_node_reject(mock_llm, mock_worker):
    mock_llm.ainvoke.return_value = MagicMock(content="""DECISION: REJECT
FEEDBACK: Simulation failed.""")
    mock_worker.read_file.side_effect = [
        json.dumps({"status": "error", "message": "collision"}),
        "Too expensive."
    ]
    
    node = CriticNode()
    state = AgentState(task="Build a part", journal="")
    
    result = await node(state)
    
    assert result["status"] == "rejected"
    assert "Simulation failed" in result["feedback"]
    assert "Critic Decision: REJECT" in result["journal"]

@pytest.mark.asyncio
async def test_critic_node_no_artifacts(mock_llm, mock_worker):
    mock_worker.read_file.side_effect = Exception("File not found")
    
    node = CriticNode()
    state = AgentState(task="Build a part", journal="")
    
    result = await node(state)
    
    # Even without artifacts, LLM decides based on journal
    assert "status" in result
    mock_llm.ainvoke.assert_called_once()