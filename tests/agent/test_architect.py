from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.architect import architect_node
from controller.agent.state import AgentState
from worker.filesystem.backend import FileInfo


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.architect.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="""# PLAN
Test Plan
# TODO
- [ ] Test Todo"""
        )
        yield instance


@pytest.fixture
def mock_worker_client():
    with patch("controller.agent.nodes.architect.WorkerClient") as mock:
        instance = mock.return_value
        instance.list_files = AsyncMock()
        instance.write_file = AsyncMock()
        instance.list_files.return_value = []
        instance.write_file.return_value = True
        yield instance


@pytest.mark.asyncio
async def test_architect_node_logic(mock_llm, mock_worker_client):
    state = AgentState(task="Build a robot")

    # Mock skills
    mock_worker_client.list_files.side_effect = [
        [FileInfo(path="/skills/skill1", name="skill1", is_dir=True)],  # /skills
        [
            FileInfo(path="suggested_skills/skill2.md", name="skill2.md", is_dir=False)
        ],  # suggested_skills
    ]

    result = await architect_node(state)

    # Check return value
    assert result.plan
    assert result.todo
    assert "Test Plan" in result.plan
    assert "Test Todo" in result.todo

    # Check file creation calls
    mock_worker_client.write_file.assert_any_call("plan.md", "Test Plan")
    mock_worker_client.write_file.assert_any_call("todo.md", "- [ ] Test Todo")

    # Check that skills were fetched
    assert mock_worker_client.list_files.call_count == 2


@pytest.mark.asyncio
async def test_architect_node_fallback(mock_llm, mock_worker_client):
    # Mock fallback response
    mock_llm.ainvoke.return_value = MagicMock(
        content="Just some text without sections"
    )

    state = AgentState(task="Build a robot")
    result = await architect_node(state)

    assert result.plan == "Just some text without sections"
    assert result.todo == "- [ ] Implement the plan"

    mock_worker_client.write_file.assert_any_call(
        "plan.md", "Just some text without sections"
    )
    mock_worker_client.write_file.assert_any_call("todo.md", "- [ ] Implement the plan")
