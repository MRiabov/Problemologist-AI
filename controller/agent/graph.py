from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph

from .nodes.architect import architect_node
from .nodes.critic import critic_node
from .nodes.engineer import engineer_node
from .nodes.sidecar import sidecar_node
from .state import AgentState, AgentStatus
from controller.config.settings import settings


def should_continue(state: AgentState) -> str:
    """Route after critic based on approval status."""
    if state.turn_count >= settings.max_agent_turns:
        return END

    if state.status == AgentStatus.APPROVED:
        return "sidecar"

    # If rejected and we haven't looped too many times
    if state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            return "architect"
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
    {"engineer": "engineer", "architect": "architect", "sidecar": "sidecar"},
)

builder.add_edge("sidecar", END)

# T026: Implement Checkpointing
memory = MemorySaver()

graph = builder.compile(checkpointer=memory)
