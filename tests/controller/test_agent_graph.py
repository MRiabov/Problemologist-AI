from unittest.mock import MagicMock, patch
import pytest
from controller.graph.agent import create_agent_graph


@patch("controller.graph.agent.get_langfuse_callback")
@patch("controller.graph.agent.get_prompt")
@patch("controller.graph.agent.ChatOpenAI")
@patch("controller.graph.agent.create_deep_agent")
def test_create_agent_graph_uses_config_prompt(
    mock_create_deep_agent, _mock_chat_openai, mock_get_prompt, mock_get_callback
):
    """
    Verifies that create_agent_graph calls get_prompt and passes it to create_deep_agent.
    """
    # Setup
    mock_backend = MagicMock()
    mock_get_prompt.return_value = "Mocked System Prompt"

    # Execute
    _agent, _callback = create_agent_graph(mock_backend)

    # Assert
    mock_get_prompt.assert_called_once_with("engineer.engineer.system")
    mock_create_deep_agent.assert_called_once()
    _args, kwargs = mock_create_deep_agent.call_args
    assert kwargs["system_prompt"] == "Mocked System Prompt"
    assert kwargs["backend"] == mock_backend


@patch("controller.graph.agent.get_langfuse_callback")
@patch("controller.graph.agent.get_prompt")
@patch("controller.graph.agent.ChatOpenAI")
@patch("controller.graph.agent.create_deep_agent")
def test_create_agent_graph_strict_prompt(
    mock_create_deep_agent, _mock_chat_openai, mock_get_prompt, _mock_get_callback
):
    """
    Verifies that create_agent_graph raises an exception if get_prompt fails.
    """
    # Setup
    mock_backend = MagicMock()
    mock_get_prompt.side_effect = Exception("Prompt not found")

    # Execute & Assert
    with pytest.raises(ValueError, match="Could not find prompt"):
        create_agent_graph(mock_backend)

    mock_get_prompt.assert_called_once_with("engineer.engineer.system")
    mock_create_deep_agent.assert_not_called()
