from langgraph.graph import StateGraph, START, END
from .state import AgentState
from .nodes.architect import architect_node
from .nodes.engineer import engineer_node
from .nodes.critic import critic_node
from .nodes.sidecar import sidecar_node

def should_continue(state: AgentState) -> str:
    """Route after critic based on approval status."""
    if state.status == "approved":
        return "sidecar"
    # If rejected and we haven't looped too many times, go back to engineer
    if state.iteration < 5:
        return "engineer"
    return "sidecar"

# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("architect", architect_node)
builder.add_node("engineer", engineer_node)
builder.add_node("critic", critic_node)
builder.add_node("sidecar", sidecar_node)

# Set the entry point and edges
builder.add_edge(START, "architect")
builder.add_edge("architect", "engineer")
builder.add_edge("engineer", "critic")

# Conditional routing from critic
builder.add_conditional_edges(
    "critic",
    should_continue,
    {
        "engineer": "engineer",
        "sidecar": "sidecar"
    }
)

builder.add_edge("sidecar", END)

graph = builder.compile()
