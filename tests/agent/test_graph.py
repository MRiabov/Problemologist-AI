import pytest
import importlib
from unittest.mock import MagicMock, AsyncMock, patch
from controller.agent.state import AgentState
import controller.agent.graph

@pytest.fixture
def fresh_graph():
    """Fixture to reload the graph after patching."""
    def _reload():
        importlib.reload(controller.agent.graph)
        return controller.agent.graph.graph
    return _reload

@pytest.mark.asyncio
async def test_full_graph_flow(fresh_graph):
    """Test the graph transitions with mocked nodes."""
    
    mock_architect = AsyncMock(return_value={"task": "Planned", "plan": "Plan", "todo": "- [ ] Task"})
    mock_engineer = AsyncMock(return_value={"journal": "Worked", "current_step": "Step"})
    mock_critic = AsyncMock(return_value={"status": "approved", "feedback": "Good"})
    mock_sidecar = AsyncMock(return_value={"journal": "Learned"})

    with patch("controller.agent.nodes.architect.architect_node", mock_architect), \
         patch("controller.agent.nodes.engineer.engineer_node", mock_engineer), \
         patch("controller.agent.nodes.critic.critic_node", mock_critic), \
         patch("controller.agent.nodes.sidecar.sidecar_node", mock_sidecar):
        
        graph = fresh_graph()
        
        config = {"configurable": {"thread_id": "test-thread"}}
        initial_state = AgentState(task="Build something")
        
        final_state = await graph.ainvoke(initial_state, config=config)
        
        assert mock_architect.called
        assert mock_engineer.called
        assert mock_critic.called
        assert mock_sidecar.called
        
        assert final_state["status"] == "approved"
        assert "Learned" in final_state["journal"]

@pytest.mark.asyncio
async def test_graph_rejection_loop(fresh_graph):
    """Test that the graph can loop back to engineer on rejection."""
    
    mock_architect = AsyncMock(return_value={"task": "Planned", "plan": "Plan", "todo": "- [ ] Task"})
    mock_engineer = AsyncMock(return_value={"journal": "Worked", "current_step": "Step"})
    
    mock_critic = AsyncMock()
    mock_critic.side_effect = [
        {"status": "code_rejected", "feedback": "Fix it", "iteration": 1},
        {"status": "approved", "feedback": "Fixed"}
    ]
    
    mock_sidecar = AsyncMock(return_value={})

    with patch("controller.agent.nodes.architect.architect_node", mock_architect), \
         patch("controller.agent.nodes.engineer.engineer_node", mock_engineer), \
         patch("controller.agent.nodes.critic.critic_node", mock_critic), \
         patch("controller.agent.nodes.sidecar.sidecar_node", mock_sidecar):
        
        graph = fresh_graph()
        
        config = {"configurable": {"thread_id": "test-loop-thread"}}
        initial_state = AgentState(task="Build something")
        
        final_state = await graph.ainvoke(initial_state, config=config)
        
        assert mock_engineer.call_count == 2
        assert mock_critic.call_count == 2
        assert final_state["status"] == "approved"