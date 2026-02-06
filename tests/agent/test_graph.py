from src.agent.graph import graph
from langgraph.graph.state import CompiledStateGraph

def test_graph_compilation():
    """Verify that the graph is compiled correctly."""
    assert graph is not None
    assert isinstance(graph, CompiledStateGraph)
    
    # Check if architect node is present
    nodes = graph.nodes
    assert "architect" in nodes
