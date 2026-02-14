import importlib
from unittest.mock import AsyncMock, patch

import pytest

import controller.agent.graph
from controller.agent.state import AgentState


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

    mock_planner = AsyncMock(
        return_value={"task": "Planned", "plan": "Plan", "todo": "- [ ] Task"}
    )
    mock_coder = AsyncMock(
        return_value={"journal": "Worked", "current_step": "Step", "todo": "- [x] Task"}
    )
    mock_electronics = AsyncMock(return_value={"journal": "Electronics worked"})
    mock_reviewer = AsyncMock(return_value={"status": "approved", "feedback": "Good"})
    mock_skills = AsyncMock(return_value={"journal": "Learned"})
    mock_cots = AsyncMock(return_value={})

    with (
        patch("controller.agent.nodes.planner.planner_node", mock_planner),
        patch("controller.agent.nodes.coder.coder_node", mock_coder),
        patch(
            "controller.agent.nodes.electronics_engineer.electronics_engineer_node",
            mock_electronics,
        ),
        patch("controller.agent.nodes.reviewer.reviewer_node", mock_reviewer),
        patch("controller.agent.nodes.skills.skills_node", mock_skills),
        patch("controller.agent.nodes.cots_search.cots_search_node", mock_cots),
    ):
        graph = fresh_graph()

        config = {"configurable": {"thread_id": "test-thread"}}
        initial_state = AgentState(task="Build something")

        final_state = await graph.ainvoke(initial_state, config=config)

        assert mock_planner.called
        assert mock_coder.called
        assert mock_reviewer.called
        assert mock_skills.called

        assert final_state["status"] == "approved"
        assert "Learned" in final_state["journal"]


@pytest.mark.asyncio
async def test_graph_rejection_loop(fresh_graph):
    """Test that the graph can loop back to engineer on rejection."""

    mock_planner = AsyncMock(
        return_value={"task": "Planned", "plan": "Plan", "todo": "- [ ] Task"}
    )
    mock_coder = AsyncMock(
        return_value={"journal": "Worked", "current_step": "Step", "todo": "- [x] Task"}
    )
    mock_electronics = AsyncMock(return_value={"journal": "Electronics worked"})

    mock_reviewer = AsyncMock()
    mock_reviewer.side_effect = [
        {"status": "code_rejected", "feedback": "Fix it", "iteration": 1},
        {"status": "approved", "feedback": "Fixed"},
    ]

    mock_skills = AsyncMock(return_value={})
    mock_cots = AsyncMock(return_value={})

    with (
        patch("controller.agent.nodes.planner.planner_node", mock_planner),
        patch("controller.agent.nodes.coder.coder_node", mock_coder),
        patch(
            "controller.agent.nodes.electronics_engineer.electronics_engineer_node",
            mock_electronics,
        ),
        patch("controller.agent.nodes.reviewer.reviewer_node", mock_reviewer),
        patch("controller.agent.nodes.skills.skills_node", mock_skills),
        patch("controller.agent.nodes.cots_search.cots_search_node", mock_cots),
    ):
        graph = fresh_graph()

        config = {"configurable": {"thread_id": "test-loop-thread"}}
        initial_state = AgentState(task="Build something")

        final_state = await graph.ainvoke(initial_state, config=config)

        assert mock_coder.call_count == 2
        assert mock_reviewer.call_count == 2
        assert final_state["status"] == "approved"
