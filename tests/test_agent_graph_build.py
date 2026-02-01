import pytest
from src.agent.graph.graph import build_graph
from src.agent.graph.state import AgentState

def test_build_graph():
    """
    Verifies that the graph is built correctly without errors.
    """
    builder = build_graph()
    # Check if nodes are present
    assert "planner" in builder.nodes
    assert "actor" in builder.nodes
    assert "critic" in builder.nodes
    assert "journaler" in builder.nodes
    assert "tools" in builder.nodes

    # Compile checking
    app = builder.compile()
    assert app is not None
