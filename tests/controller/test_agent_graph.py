from unittest.mock import MagicMock, patch
import pytest
from controller.graph.agent import create_agent_graph

@patch("controller.graph.agent.get_prompt")
@patch("controller.graph.agent.ChatOpenAI")
@patch("controller.graph.agent.create_react_agent")
@patch("controller.graph.agent.create_fs_tools")
def test_create_agent_graph_uses_config_prompt(
    mock_create_fs_tools,
    mock_create_react_agent,
    mock_chat_openai,
    mock_get_prompt
):
    """
    Verifies that create_agent_graph calls get_prompt and passes it to create_react_agent.
    """
    # Setup
    mock_fs_middleware = MagicMock()
    mock_get_prompt.return_value = "Mocked System Prompt"
    mock_create_fs_tools.return_value = []
    
    # Execute
    create_agent_graph(mock_fs_middleware)
    
    # Assert
    mock_get_prompt.assert_called_once_with("cad_agent.planner.system")
    mock_create_react_agent.assert_called_once()
    args, kwargs = mock_create_react_agent.call_args
    assert kwargs["prompt"] == "Mocked System Prompt"

@patch("controller.graph.agent.get_prompt")
@patch("controller.graph.agent.ChatOpenAI")
@patch("controller.graph.agent.create_react_agent")
@patch("controller.graph.agent.create_fs_tools")
def test_create_agent_graph_fallback_prompt(
    mock_create_fs_tools,
    mock_create_react_agent,
    mock_chat_openai,
    mock_get_prompt
):
    """
    Verifies that create_agent_graph falls back to a default prompt if get_prompt fails.
    """
    # Setup
    mock_fs_middleware = MagicMock()
    mock_get_prompt.side_effect = Exception("Prompt not found")
    mock_create_fs_tools.return_value = []
    
    # Execute
    create_agent_graph(mock_fs_middleware)
    
    # Assert
    mock_get_prompt.assert_called_once_with("cad_agent.planner.system")
    mock_create_react_agent.assert_called_once()
    args, kwargs = mock_create_react_agent.call_args
    assert "You are a coding agent" in kwargs["prompt"]
