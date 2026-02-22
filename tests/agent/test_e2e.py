import importlib
from unittest.mock import AsyncMock, patch

import pytest

import controller.agent.graph
from controller.agent.state import AgentState


@pytest.fixture
def fresh_graph():
    def _reload():
        importlib.reload(controller.agent.graph)
        return controller.agent.graph.graph

    return _reload


@pytest.mark.asyncio
async def test_smoke_e2e(fresh_graph):
    """Smoke test: Create a 10x10x10 cube."""

    # Mocking node implementations to avoid actual LLM and Worker calls
    mock_planner = AsyncMock(
        return_value={
            "task": "Create a 10x10x10 cube",
            "plan": "1. Import box. 2. Create box.",
            "todo": "- [ ] Create cube",
        }
    )

    mock_coder = AsyncMock(
        return_value={
            "journal": "Created cube successfully",
            "current_step": "Create cube",
            "todo": "- [x] Create cube",
            "status": "approved",  # reviewer will check this
        }
    )

    mock_electronics_engineer = AsyncMock(
        return_value={
            "journal": "No electronics needed for a simple cube.",
        }
    )

    mock_reviewer = AsyncMock(
        return_value={"status": "approved", "feedback": "Perfect cube."}
    )

    mock_skills = AsyncMock(return_value={"journal": "Learned about cubes."})

    with (
        patch("controller.agent.nodes.planner.planner_node", mock_planner),
        patch("controller.agent.nodes.electronics_planner.electronics_planner_node", mock_planner),
        patch("controller.agent.nodes.coder.coder_node", mock_coder),
        patch(
            "controller.agent.nodes.electronics_engineer.electronics_engineer_node",
            mock_electronics_engineer,
        ),
        patch("controller.agent.nodes.plan_reviewer.plan_reviewer_node", mock_reviewer),
        patch("controller.agent.nodes.execution_reviewer.execution_reviewer_node", mock_reviewer),
        patch("controller.agent.nodes.skills.skills_node", mock_skills),
    ):
        graph = fresh_graph()

        config = {"configurable": {"thread_id": "smoke-test"}}
        initial_state = AgentState(task="Create a 10x10x10 cube")

        # We collect all chunks from the stream
        events = []
        async for event in graph.astream(initial_state, config=config):
            events.append(event)

        # Verify the sequence of nodes
        node_sequence = [list(e.keys())[0] for e in events]
        assert "planner" in node_sequence
        assert "coder" in node_sequence
        assert "electronics_engineer" in node_sequence
        assert "plan_reviewer" in node_sequence
        assert "execution_reviewer" in node_sequence
        assert "skills" in node_sequence

        # Verify last state
        last_event = events[-1]
        assert "skills" in last_event
        assert "Learned about cubes" in last_event["skills"]["journal"]
