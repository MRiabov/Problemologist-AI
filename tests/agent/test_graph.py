from src.agent.graph import graph
from langgraph.graph.state import CompiledStateGraph

<<<<<<< HEAD

=======
>>>>>>> 002-vlm-cad-agent-WP03
def test_graph_compilation():
    """Verify that the graph is compiled correctly."""
    assert graph is not None
    assert isinstance(graph, CompiledStateGraph)
<<<<<<< HEAD

=======
    
>>>>>>> 002-vlm-cad-agent-WP03
    # Check if architect node is present
    nodes = graph.nodes
    assert "architect" in nodes
