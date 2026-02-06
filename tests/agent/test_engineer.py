import pytest
from unittest.mock import patch, AsyncMock
from langchain_core.messages import AIMessage
from src.worker.api.schema import ExecuteResponse
from src.agent.nodes.engineer import EngineerNode
from src.agent.state import AgentState


@pytest.fixture
def mock_llm():
    with patch("src.agent.nodes.engineer.ChatOpenAI") as mock:
        instance = mock.return_value
        # Mock bind_tools to return the instance itself (simplified)
        instance.bind_tools.return_value = instance
        yield instance


@pytest.fixture
def mock_worker():
    with patch("src.agent.nodes.engineer.WorkerClient") as mock:
        instance = mock.return_value
        instance.read_file = AsyncMock(return_value="file content")
        instance.list_files = AsyncMock(return_value=[{"name": "file.txt"}])
        instance.execute_python = AsyncMock(
            return_value=ExecuteResponse(stdout="hello", stderr="", exit_code=0)
        )
        yield instance


@pytest.mark.asyncio
async def test_engineer_node_direct_success(mock_llm, mock_worker):
    """Test standard flow: LLM immediately generates code."""
    # Setup LLM response
    mock_llm.ainvoke = AsyncMock(
        return_value=AIMessage(content="```python\nprint('hello')\n```")
    )

    node = EngineerNode()
    state = AgentState(
        todo="- [ ] Step 1\n- [ ] Step 2",
        plan="The plan",
        journal="Old logs",
        iteration=0,
    )

    result = await node(state)

    assert "Step 1" in result["current_step"]
    assert "- [x] Step 1" in result["todo"]
    assert "Successfully executed step: Step 1" in result["journal"]
    mock_worker.execute_python.assert_called_once()


@pytest.mark.asyncio
async def test_engineer_node_tool_usage(mock_llm, mock_worker):
    """Test flow: LLM uses tool (ls), sees result, then generates code."""
    # Setup LLM responses:
    # 1. Tool call
    # 2. Code generation (after tool result)
    response_1 = AIMessage(
        content="",
        tool_calls=[{"name": "list_files", "args": {"path": "/"}, "id": "call_1"}],
    )
    response_2 = AIMessage(content="```python\nprint('hello')\n```")

    mock_llm.ainvoke = AsyncMock(side_effect=[response_1, response_2])

    node = EngineerNode()
    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="", iteration=0)

    result = await node(state)

    # Verify tool was called
    mock_worker.list_files.assert_called_with("/")

    # Verify execution happened
    mock_worker.execute_python.assert_called_once()
    assert "- [x] Step 1" in result["todo"]


@pytest.mark.asyncio
async def test_engineer_node_retry_logic(mock_llm, mock_worker):
    """Test flow: Code fails -> Error fed back -> New code -> Success."""
    mock_llm.ainvoke = AsyncMock(
        return_value=AIMessage(content="```python\nprint('fail')\n```")
    )

    # Worker fails first, then succeeds
    mock_worker.execute_python.side_effect = [
        ExecuteResponse(stdout="", stderr="SyntaxError", exit_code=1),
        ExecuteResponse(stdout="fixed", stderr="", exit_code=0),
    ]

    node = EngineerNode()
    state = AgentState(todo="- [ ] Step 1", plan="The plan", journal="", iteration=0)

    result = await node(state)

    assert mock_worker.execute_python.call_count == 2
    assert "Execution failed (Attempt 1): SyntaxError" in result["journal"]
    assert "Successfully executed step: Step 1" in result["journal"]
