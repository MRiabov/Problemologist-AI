from langgraph.graph import StateGraph, START, END
from .state import AgentState
from .nodes.architect import architect_node
<<<<<<< HEAD
<<<<<<< HEAD


# Dummy node for future nodes
def dummy_node(state: AgentState) -> AgentState:
    return state


=======
from .nodes.engineer import engineer_node

# Placeholder for Critic
def critic_node(state: AgentState) -> AgentState:
    return state

>>>>>>> 002-vlm-cad-agent-WP03
=======
from .nodes.engineer import engineer_node
from .nodes.critic import critic_node

def should_continue(state: AgentState) -> str:
    """Route after critic based on approval status."""
    if state.status == "approved":
        return END
    # If rejected and we haven't looped too many times, go back to engineer
    if state.iteration < 5:
        return "engineer"
    return END

>>>>>>> 002-vlm-cad-agent-WP04
# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("architect", architect_node)
<<<<<<< HEAD
<<<<<<< HEAD
builder.add_node("dummy", dummy_node)

# Set the entry point and edges
builder.add_edge(START, "architect")
builder.add_edge("architect", "dummy")
builder.add_edge("dummy", END)
=======
=======
>>>>>>> 002-vlm-cad-agent-WP04
builder.add_node("engineer", engineer_node)
builder.add_node("critic", critic_node)

# Set the entry point and edges
builder.add_edge(START, "architect")
builder.add_edge("architect", "engineer")
builder.add_edge("engineer", "critic")
<<<<<<< HEAD
builder.add_edge("critic", END)
>>>>>>> 002-vlm-cad-agent-WP03
=======

# Conditional routing from critic
builder.add_conditional_edges(
    "critic",
    should_continue,
    {
        "engineer": "engineer",
        END: END
    }
)
>>>>>>> 002-vlm-cad-agent-WP04

graph = builder.compile()
