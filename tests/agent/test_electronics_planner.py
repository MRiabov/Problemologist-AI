from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.electronics_planner import electronics_planner_node
from controller.agent.state import AgentState, AgentStatus


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="# Mock content")
        instance.exists = AsyncMock(return_value=True)
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.electronics_planner.record_worker_events")
@patch("controller.agent.nodes.electronics_planner.ElectronicsPlannerNode._run_program")
@patch("controller.agent.nodes.electronics_planner.SharedNodeContext")
async def test_electronics_planner_node_logic(
    mock_ctx_cls, mock_run_program, mock_record_events, mock_worker
):
    mock_pm = MagicMock()
    mock_pm.render.return_value = "Rendered prompt"

    mock_fs = MagicMock()
    mock_fs.read_file = AsyncMock(return_value="plan.md content")
    mock_fs.write_file = AsyncMock(return_value=True)

    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext(
        worker_light_url="http://worker",
        session_id="test-session",
        pm=mock_pm,
        dspy_lm=MagicMock(),
        worker_client=MagicMock(),
        fs=mock_fs,
    )
    mock_ctx_cls.create.return_value = mock_ctx

    # Mock _run_program return value: (prediction, artifacts, journal_entry)
    mock_prediction = MagicMock(summary="Finished electronics planning.")
    mock_run_program.return_value = (
        mock_prediction,
        {"plan.md": "New electrical plan", "todo.md": "- [ ] Wire it"},
        "\n[Electronics Planner] Summary",
    )

    state = AgentState(
        task="Build robot", plan="Mechanical plan", session_id="test-session"
    )

    result = await electronics_planner_node(state)

    assert "New electrical plan" in result.plan
    assert "- [ ] Wire it" in result.todo
    assert "[Electronics Planner] Summary" in result.journal
    assert any(
        isinstance(m, AIMessage)
        and "Electronics Plan summary: Finished electronics planning." in m.content
        for m in result.messages
    )


@pytest.mark.asyncio
@patch("controller.agent.nodes.electronics_planner.record_worker_events")
@patch("controller.agent.nodes.electronics_planner.ElectronicsPlannerNode._run_program")
@patch("controller.agent.nodes.electronics_planner.SharedNodeContext")
async def test_electronics_planner_node_failure(
    mock_ctx_cls, mock_run_program, mock_record_events, mock_worker
):
    mock_pm = MagicMock()
    mock_pm.render.return_value = "Rendered prompt"

    from controller.agent.nodes.base import SharedNodeContext

    mock_ctx = SharedNodeContext(
        worker_light_url="http://worker",
        session_id="test-session",
        pm=mock_pm,
        dspy_lm=MagicMock(),
        worker_client=MagicMock(),
        fs=MagicMock(),
    )
    mock_ctx_cls.create.return_value = mock_ctx

    # Mock _run_program return value: (None, {}, journal_entry)
    mock_run_program.return_value = (None, {}, "\nMax retries reached.")

    state = AgentState(task="Build robot", session_id="test-session")

    result = await electronics_planner_node(state)

    assert result.status == AgentStatus.FAILED
    assert "Max retries reached." in result.journal
