from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.nodes.coder import coder_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.base.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = MagicMock(
            content="```python\nprint('hello')\n```"
        )
        yield instance


from worker.api.schema import ExecuteResponse


@pytest.fixture
def mock_worker():
    with patch("controller.agent.nodes.base.WorkerClient") as mock:
        instance = mock.return_value
        instance.execute_python = AsyncMock()
        instance.execute_python.return_value = ExecuteResponse(
            stdout="hello", stderr="", exit_code=0
        )
        instance.read_file = AsyncMock()
        # Mocking valid contents for coder validation
        instance.read_file.side_effect = lambda path: {
            "plan.md": "## 1. Solution Overview\nTest\n## 2. Parts List\n- Part A\n## 3. Assembly Strategy\n1. Step 1\n## 4. Cost & Weight Budget\n- $10\n## 5. Risk Assessment\n- Risk 1",
            "todo.md": "- [ ] Step 1\n- [ ] Step 2",
            "objectives.yaml": "objectives: {goal_zone: {min: [0,0,0], max: [1,1,1]}, build_zone: {min: [0,0,0], max: [1,1,1]}}\nsimulation_bounds: {min: [0,0,0], max: [1,1,1]}\nmoved_object: {initial_pos: [0,0,0]}\nconstraints: {max_unit_cost: 100, max_weight: 100}",
        }.get(path, "content")
        yield instance


@pytest.fixture
def mock_record_events():
    with (
        patch(
            "controller.middleware.remote_fs.record_worker_events",
            new_callable=AsyncMock,
        ) as mock2,
    ):
        yield mock2


@pytest.mark.asyncio
@patch("worker.utils.file_validation.validate_node_output", return_value=(True, []))
async def test_engineer_node_success(
    mock_validate, mock_llm, mock_worker, mock_record_events
):
    from controller.agent.nodes.coder import coder_node

    state = AgentState(
        todo="- [ ] Step 1\n- [ ] Step 2", plan="The plan", journal="Old logs"
    )

    result = await coder_node(state)

    assert "Step 1" in result.current_step
    assert "- [x] Step 1" in result.todo
    assert "Successfully executed step: Step 1" in result.journal
    mock_worker.execute_python.assert_called_once()


@pytest.mark.asyncio
@patch("worker.utils.file_validation.validate_node_output", return_value=(True, []))
async def test_engineer_node_retry_then_success(
    mock_validate, mock_llm, mock_worker, mock_record_events
):
    # Mock failure then success
    mock_worker.execute_python.side_effect = [
        ExecuteResponse(stdout="", stderr="SyntaxError", exit_code=1),
        ExecuteResponse(stdout="fixed", stderr="", exit_code=0),
    ]

    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="")

    result = await coder_node(state)

    assert "- [x] Step 1" in result.todo
    assert "Execution failed (Attempt 1): SyntaxError" in result.journal
    assert "Successfully executed step: Step 1" in result.journal
    assert mock_worker.execute_python.call_count == 2


@pytest.mark.asyncio
async def test_engineer_node_all_fail(mock_llm, mock_worker, mock_record_events):
    mock_worker.execute_python.return_value = ExecuteResponse(
        stdout="", stderr="Persistent Error", exit_code=1
    )

    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="")

    result = await coder_node(state)

    assert result.iteration > 0
    assert "Failed to complete step after 3 retries" in result.journal
    assert mock_worker.execute_python.call_count == 3
