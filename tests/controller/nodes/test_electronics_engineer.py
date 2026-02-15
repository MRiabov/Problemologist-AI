import pytest
from unittest.mock import MagicMock, AsyncMock, patch, call
from controller.agent.nodes.electronics_engineer import ElectronicsEngineerNode
from controller.agent.state import AgentState, AgentStatus
from langchain_core.messages import AIMessage

@pytest.fixture
def mock_worker_client():
    return MagicMock()

@pytest.fixture
def mock_fs_middleware():
    mock = MagicMock()
    mock.read_file = AsyncMock()
    mock.run_command = AsyncMock()
    return mock

@pytest.fixture
def mock_llm():
    mock = MagicMock()
    mock.ainvoke = AsyncMock()
    return mock

@pytest.mark.asyncio
async def test_electronics_engineer_logs_file_errors(mock_worker_client, mock_fs_middleware, mock_llm):
    # Setup mocks
    with patch("controller.agent.nodes.electronics_engineer.WorkerClient", return_value=mock_worker_client), \
         patch("controller.agent.nodes.electronics_engineer.RemoteFilesystemMiddleware", return_value=mock_fs_middleware), \
         patch("controller.agent.nodes.electronics_engineer.ChatOpenAI", return_value=mock_llm), \
         patch("controller.agent.nodes.electronics_engineer.record_worker_events", new_callable=AsyncMock), \
         patch("controller.agent.nodes.electronics_engineer.settings") as mock_settings, \
         patch("controller.agent.nodes.electronics_engineer.logger") as mock_logger:

        mock_settings.llm_model = "gpt-4"
        mock_settings.spec_001_api_url = "http://worker:8001"
        mock_settings.default_session_id = "default-session"

        node = ElectronicsEngineerNode(worker_url="http://worker:8001", session_id="test-session")
        node.fs = mock_fs_middleware
        node.llm = mock_llm

        # Mock read_file to raise Exception
        mock_fs_middleware.read_file.side_effect = Exception("File read error")

        # Mock LLM to return empty content so we return early
        mock_llm.ainvoke.return_value = AIMessage(content="")

        # Create dummy state
        state = AgentState(
            session_id="test-session",
            task="Design a circuit",
            plan="Plan",
            todo="Todo",
            journal="",
            turn_count=0,
            status=AgentStatus.EXECUTING
        )

        # Execute node
        await node(state)

        # Verify that warning/error WAS logged (fixed behavior)
        assert mock_logger.warning.call_count >= 1

@pytest.mark.asyncio
async def test_electronics_engineer_retries_execution(mock_worker_client, mock_fs_middleware, mock_llm):
    # Setup mocks
    with patch("controller.agent.nodes.electronics_engineer.WorkerClient", return_value=mock_worker_client), \
         patch("controller.agent.nodes.electronics_engineer.RemoteFilesystemMiddleware", return_value=mock_fs_middleware), \
         patch("controller.agent.nodes.electronics_engineer.ChatOpenAI", return_value=mock_llm), \
         patch("controller.agent.nodes.electronics_engineer.record_worker_events", new_callable=AsyncMock), \
         patch("controller.agent.nodes.electronics_engineer.settings") as mock_settings, \
         patch("controller.agent.nodes.electronics_engineer.logger") as mock_logger:

        mock_settings.llm_model = "gpt-4"
        mock_settings.spec_001_api_url = "http://worker:8001"
        mock_settings.default_session_id = "default-session"

        node = ElectronicsEngineerNode(worker_url="http://worker:8001", session_id="test-session")
        node.fs = mock_fs_middleware
        node.llm = mock_llm

        # Mock read_file success
        mock_fs_middleware.read_file.return_value = "electronics_requirements: true"

        # Mock LLM to return code
        code_block = "```python\nprint('Hello')\n```"
        mock_llm.ainvoke.return_value = AIMessage(content=code_block)

        # Mock run_command to fail consistently
        mock_fs_middleware.run_command.return_value = {"exit_code": 1, "stderr": "Error", "stdout": ""}

        state = AgentState(
            session_id="test-session",
            task="Design a circuit",
            plan="Plan",
            todo="Todo",
            journal="",
            turn_count=0,
            status=AgentStatus.EXECUTING
        )

        # Execute node
        await node(state)

        # Verify run_command was called 3 times (retries)
        assert mock_fs_middleware.run_command.call_count == 3

        # Verify LLM was invoked 3 times (to fix code)
        assert mock_llm.ainvoke.call_count == 3
