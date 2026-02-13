import pytest
from unittest.mock import MagicMock, patch, AsyncMock
from controller.agent.nodes.electronics_engineer import ElectronicsEngineerNode
from controller.agent.state import AgentState
from langchain_core.messages import AIMessage

@pytest.mark.asyncio
async def test_electronics_engineer_node_success():
    # Mock LLM response
    mock_llm = MagicMock()
    mock_llm.ainvoke = AsyncMock(return_value=AIMessage(content="```python\n# Circuit design\nprint('Circuit implementation done')\n```"))
    
    # Mock WorkerClient/Filesystem
    mock_fs = MagicMock()
    mock_fs.read_file = AsyncMock(return_value="electronics_requirements: {}")
    mock_fs.run_command = AsyncMock(return_value={"exit_code": 0, "stdout": "Success", "events": []})
    
    state = AgentState(
        task="Test Task",
        plan="Plan",
        session_id="test-session"
    )
    
    with patch("controller.agent.nodes.electronics_engineer.ChatOpenAI", return_value=mock_llm):
        with patch("controller.agent.nodes.electronics_engineer.RemoteFilesystemMiddleware", return_value=mock_fs):
            node = ElectronicsEngineerNode()
            new_state = await node(state)
            
            assert "Circuit and wiring implementation successful" in new_state.journal
            assert new_state.turn_count == 1
            mock_fs.run_command.assert_called_once()

@pytest.mark.asyncio
async def test_electronics_engineer_node_failure():
    mock_llm = MagicMock()
    mock_llm.ainvoke = AsyncMock(return_value=AIMessage(content="```python\n# Faulty code\n```"))
    
    mock_fs = MagicMock()
    mock_fs.read_file = AsyncMock(return_value="electronics_requirements: {}")
    mock_fs.run_command = AsyncMock(return_value={"exit_code": 1, "stderr": "Short circuit error", "stdout": ""})
    
    state = AgentState(task="Test Task", plan="Plan", session_id="test-session")
    
    with patch("controller.agent.nodes.electronics_engineer.ChatOpenAI", return_value=mock_llm):
        with patch("controller.agent.nodes.electronics_engineer.RemoteFilesystemMiddleware", return_value=mock_fs):
            node = ElectronicsEngineerNode()
            new_state = await node(state)
            
            assert "Circuit implementation failed" in new_state.journal
            assert "Short circuit error" in new_state.journal
