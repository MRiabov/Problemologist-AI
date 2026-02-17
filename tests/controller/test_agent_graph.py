from unittest.mock import MagicMock, patch

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
    # We check the nodes to ensure it's the right graph,
    # as object identity might fail due to multiple imports in some environments
    assert "planner" in agent.nodes
    assert "coder" in agent.nodes
    assert "reviewer" in agent.nodes
    assert callback == mock_callback


@patch("controller.graph.agent.get_langfuse_callback")
def test_create_agent_graph_returns_benchmark_graph(mock_get_callback):
    """
    Verifies that create_agent_graph returns the benchmark graph.
    """
    # Setup
    mock_callback = MagicMock()
    mock_get_callback.return_value = mock_callback

    # Execute
    agent, _ = create_agent_graph(agent_name="benchmark")

    # Assert
    assert "planner" in agent.nodes
    assert "coder" in agent.nodes
    assert "reviewer" in agent.nodes
    # Benchmark graph also has cots_search and skills
    assert "cots_search" in agent.nodes
    assert "skills" in agent.nodes
