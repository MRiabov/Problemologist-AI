import pytest
import importlib
from unittest.mock import MagicMock, AsyncMock, patch
from src.agent.state import AgentState
import src.agent.graph

@pytest.fixture
def fresh_graph():
    def _reload():
        importlib.reload(src.agent.graph)
        return src.agent.graph.graph
    return _reload

@pytest.mark.asyncio
async def test_smoke_e2e(fresh_graph):
    """Smoke test: Create a 10x10x10 cube."""
    
    # Mocking node implementations to avoid actual LLM and Worker calls
    mock_architect = AsyncMock(return_value={
        "task": "Create a 10x10x10 cube",
        "plan": "1. Import box. 2. Create box.",
        "todo": "- [ ] Create cube"
    })
    
    mock_engineer = AsyncMock(return_value={
        "journal": "Created cube successfully",
        "current_step": "Create cube",
        "todo": "- [x] Create cube"
    })
    
    mock_critic = AsyncMock(return_value={
        "status": "approved",
        "feedback": "Perfect cube."
    })
    
    mock_sidecar = AsyncMock(return_value={"journal": "Learned about cubes."})

    with patch("src.agent.nodes.architect.architect_node", mock_architect), \
         patch("src.agent.nodes.engineer.engineer_node", mock_engineer), \
         patch("src.agent.nodes.critic.critic_node", mock_critic), \
         patch("src.agent.nodes.sidecar.sidecar_node", mock_sidecar):
        
        graph = fresh_graph()
        
        config = {"configurable": {"thread_id": "smoke-test"}}
        initial_state = AgentState(task="Create a 10x10x10 cube")
        
        # We collect all chunks from the stream
        events = []
        async for event in graph.astream(initial_state, config=config):
            events.append(event)
        
        # Verify the sequence of nodes
        node_sequence = [list(e.keys())[0] for e in events]
        assert "architect" in node_sequence
        assert "engineer" in node_sequence
        assert "critic" in node_sequence
        assert "sidecar" in node_sequence
        
        # Verify last state
        last_event = events[-1]
        assert "sidecar" in last_event
        assert "Learned about cubes" in last_event["sidecar"]["journal"]
