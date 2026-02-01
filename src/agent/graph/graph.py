from langgraph.graph import StateGraph, END
from src.agent.graph.state import AgentState


def build_graph():
    """
    Constructs the LangGraph for the VLM CAD Agent.
    Nodes and edges will be implemented in subsequent work packages.
    """
    builder = StateGraph(AgentState)

    # Current implementation is just a skeleton.
    # In WP03, we will add:
    # builder.add_node("planner", planner_node)
    # builder.add_node("executor", executor_node)
    # builder.add_node("reflector", reflector_node)

    # builder.set_entry_point(...)

    return builder
