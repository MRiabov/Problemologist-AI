from controller.graph.agent import create_agent_graph


def test_create_agent_graph_returns_engineering_graph():
    """
    Verifies that create_agent_graph returns the engineering graph.
    """
    # Execute
    agent, callback = create_agent_graph(agent_name="engineer")

    # Assert
    assert "planner" in agent.nodes
    assert "coder" in agent.nodes
    assert "plan_reviewer" in agent.nodes
    assert "execution_reviewer" in agent.nodes
    assert callback is None


def test_create_agent_graph_returns_benchmark_graph():
    """
    Verifies that create_agent_graph returns the benchmark graph.
    """
    # Execute
    agent, _ = create_agent_graph(agent_name="benchmark")

    # Assert
    assert "planner" in agent.nodes
    assert "coder" in agent.nodes
    assert "reviewer" in agent.nodes
    # Benchmark graph also has cots_search and skills
    assert "cots_search" in agent.nodes
    assert "skills" in agent.nodes
