from unittest.mock import MagicMock, patch

import pytest

from controller.graph.agent import create_agent_graph


@patch("controller.graph.agent.get_langfuse_callback")
def test_create_agent_graph_returns_engineering_graph(mock_get_callback):
    """
    Verifies that create_agent_graph returns the engineering graph and callback.
    """
    # Setup
    mock_callback = MagicMock()
    mock_get_callback.return_value = mock_callback

    # Execute
    agent, callback = create_agent_graph(agent_name="engineer")

    # Assert
    from controller.agent.graph import graph as engineering_graph

    assert agent == engineering_graph
    assert callback == mock_callback
    mock_get_callback.assert_called_once_with(
        trace_id=None, name="engineer", session_id=None
    )


def test_create_agent_graph_returns_benchmark_graph():
    """
    Verifies that create_agent_graph returns the benchmark graph.
    """
    # Execute
    agent, _callback = create_agent_graph(agent_name="benchmark_gen")

    # Assert
    # We check if it's a CompiledGraph from langgraph
    from langgraph.graph.graph import CompiledGraph

    assert isinstance(agent, CompiledGraph)
