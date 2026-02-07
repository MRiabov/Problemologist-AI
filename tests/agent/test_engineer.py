from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.engineer import EngineerNode
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.engineer.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="```python\nprint('hello')\n```"
        )
        yield instance


from worker.api.schema import ExecuteResponse


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.engineer.WorkerClient") as mock:
        instance = mock.return_value
        instance.execute_python = AsyncMock()
        instance.execute_python.return_value = ExecuteResponse(
            stdout="hello", stderr="", exit_code=0
        )
        yield instance


@pytest.mark.asyncio
async def test_engineer_node_success(mock_llm, mock_worker):
    node = EngineerNode()
    state = AgentState(
        todo="- [ ] Step 1\n- [ ] Step 2", plan="The plan", journal="Old logs"
    )

    result = await node(state)

    assert "Step 1" in result.current_step
    assert "- [x] Step 1" in result.todo
    assert "Successfully executed step: Step 1" in result.journal
    mock_worker.execute_python.assert_called_once()


@pytest.mark.asyncio
async def test_engineer_node_retry_then_success(mock_llm, mock_worker):
    # Mock failure then success
    mock_worker.execute_python.side_effect = [
        ExecuteResponse(stdout="", stderr="SyntaxError", exit_code=1),
        ExecuteResponse(stdout="fixed", stderr="", exit_code=0),
    ]

    node = EngineerNode()
    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="")

    result = await node(state)

    assert "- [x] Step 1" in result.todo
    assert "Execution failed (Attempt 1): SyntaxError" in result.journal
    assert "Successfully executed step: Step 1" in result.journal
    assert mock_worker.execute_python.call_count == 2


@pytest.mark.asyncio
async def test_engineer_node_all_fail(mock_llm, mock_worker):
    mock_worker.execute_python.return_value = ExecuteResponse(
        stdout="", stderr="Persistent Error", exit_code=1
    )

    node = EngineerNode()
    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="")

    result = await node(state)

    assert result.iteration > 0
    assert "Failed to complete step after 3 retries" in result.journal
    assert mock_worker.execute_python.call_count == 3
