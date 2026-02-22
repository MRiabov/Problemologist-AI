from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.electronics_engineer import electronics_engineer_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="# Mock content")
        instance.exists = AsyncMock(return_value=True)
        yield instance


@pytest.mark.asyncio
@patch("controller.agent.nodes.electronics_engineer.record_worker_events")
@patch(
    "controller.agent.nodes.electronics_engineer.ElectronicsEngineerNode._run_program"
)
@patch("controller.agent.nodes.electronics_engineer.SharedNodeContext")
async def test_electronics_engineer_node_success(
    mock_ctx_cls, mock_run_program, mock_record_events, mock_worker
):
    mock_pm = MagicMock()
    mock_pm.render.return_value = "Rendered prompt"

    mock_fs = MagicMock()
    mock_fs.read_file = AsyncMock(return_value="assembly_definition.yaml content")
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
    mock_prediction = MagicMock(journal="Electronics work done.")
    mock_run_program.return_value = (
        mock_prediction,
        {"script.py": "print('hello')"},
        "\n[Electronics] Summary",
    )

    state = AgentState(
        task="Build robot",
        todo="- [ ] Step 1\n- [ ] Wire it",
        plan="The plan",
        session_id="test-session",
    )

    result = await electronics_engineer_node(state)

    assert "- [x] Wire it" in result.todo
    assert "[Electronics] Summary" in result.journal
    assert any(
        isinstance(m, AIMessage)
        and "Electronics summary: Electronics work done." in m.content
        for m in result.messages
    )


@pytest.mark.asyncio
@patch("controller.agent.nodes.electronics_engineer.record_worker_events")
@patch(
    "controller.agent.nodes.electronics_engineer.ElectronicsEngineerNode._run_program"
)
@patch("controller.agent.nodes.electronics_engineer.SharedNodeContext")
async def test_electronics_engineer_node_failure(
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

    state = AgentState(
        task="Build robot",
        todo="- [ ] Step 1\n- [ ] Wire it",
        session_id="test-session",
    )

    result = await electronics_engineer_node(state)

    assert "Max retries reached." in result.journal
