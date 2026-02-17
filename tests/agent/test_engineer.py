from unittest.mock import AsyncMock, patch

import pytest
from langchain_core.messages import AIMessage

from controller.agent.nodes.coder import coder_node
from controller.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("controller.agent.nodes.base.ChatOpenAI") as mock:
        instance = mock.return_value
        instance.ainvoke = AsyncMock()
        instance.ainvoke.return_value = AIMessage(
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
            "script.py": "print('hello')",
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
@patch("controller.agent.nodes.coder.dspy.CodeAct")
@patch("worker.utils.file_validation.validate_node_output", return_value=(True, []))
async def test_engineer_node_success(
    mock_validate, mock_codeact_cls, mock_llm, mock_worker, mock_record_events
):
    # Mock DSPy Program
    mock_program = MagicMock()
    mock_program.return_value = MagicMock(journal="Step done.")
    mock_codeact_cls.return_value = mock_program

    state = AgentState(
        todo="- [ ] Step 1\n- [ ] Step 2", plan="The plan", journal="Old logs",
        session_id="test-session"
    )

    result = await coder_node(state)

    assert "Step 1" in result.current_step
    assert "- [x] Step 1" in result.todo
    assert "[Coder] Step done." in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.coder.dspy.CodeAct")
@patch("worker.utils.file_validation.validate_node_output", return_value=(True, []))
async def test_engineer_node_retry_then_success(
    mock_validate, mock_codeact_cls, mock_llm, mock_worker, mock_record_events
):
    # Mock DSPy Program
    mock_program = MagicMock()
    # First call fails (exception), second succeeds
    mock_program.side_effect = [
        Exception("ExecutionError"),
        MagicMock(journal="Fixed."),
    ]
    mock_codeact_cls.return_value = mock_program

    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="", session_id="test-session")

    result = await coder_node(state)

    assert "- [x] Step 1" in result.todo
    assert "[System Error] ExecutionError" in str(result.journal) # Error is logged, retry happens
    assert "[Coder] Fixed." in result.journal


@pytest.mark.asyncio
@patch("controller.agent.nodes.coder.dspy.CodeAct")
async def test_engineer_node_all_fail(
    mock_codeact_cls, mock_llm, mock_worker, mock_record_events
):
    # Mock DSPy Program that always fails
    mock_program = MagicMock()
    mock_program.side_effect = Exception("Persistent Error")
    mock_codeact_cls.return_value = mock_program

    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="", session_id="test-session")

    result = await coder_node(state)

    assert result.iteration > 0
    assert "Max retries reached." in result.journal
