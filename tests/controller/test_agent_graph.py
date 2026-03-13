from controller.graph.agent import create_agent_graph
from shared.enums import AgentName


def test_create_agent_graph_returns_engineering_graph():
    """
    Verifies that create_agent_graph returns the engineering graph.
    """
    # Execute
    agent, callback = create_agent_graph(agent_name=AgentName.ENGINEER_CODER)

    # Assert
    assert AgentName.ENGINEER_PLANNER in agent.nodes
    assert AgentName.ENGINEER_CODER in agent.nodes
    assert AgentName.ENGINEER_PLAN_REVIEWER in agent.nodes
    assert "execution_reviewer" in agent.nodes
    assert callback is None


def test_create_agent_graph_returns_benchmark_graph():
    """
    Verifies that create_agent_graph returns the benchmark graph.
    """
    # Execute
    agent, _ = create_agent_graph(agent_name=AgentName.BENCHMARK_PLANNER)

    # Assert
    assert AgentName.BENCHMARK_PLANNER in agent.nodes
    assert AgentName.BENCHMARK_PLAN_REVIEWER in agent.nodes
    assert AgentName.BENCHMARK_CODER in agent.nodes
    assert AgentName.BENCHMARK_REVIEWER in agent.nodes
    # Benchmark graph also has cots_search and skills
    assert AgentName.COTS_SEARCH in agent.nodes
    assert "skills" in agent.nodes
