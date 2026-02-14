from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph

from controller.config.settings import settings

from .nodes.planner import planner_node
from .nodes.reviewer import reviewer_node
from .nodes.coder import coder_node
from .nodes.skills import skills_node
from .nodes.cots_search import cots_search_node
from .state import AgentState, AgentStatus


def should_continue(state: AgentState) -> str:
    """Route after reviewer based on approval status."""
    if state.turn_count >= settings.max_agent_turns:
        return "skills"

    if state.status == AgentStatus.APPROVED:
        return "skills"

    # If rejected and we haven't looped too many times
    if state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            return "planner"
        return "coder"

    return "skills"


# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("planner", planner_node)
builder.add_node("coder", coder_node)
builder.add_node("reviewer", reviewer_node)
builder.add_node("cots_search", cots_search_node)
builder.add_node("skills", skills_node)

# Set the entry point and edges
builder.add_edge(START, "planner")
builder.add_edge("planner", "coder")
builder.add_edge("coder", "reviewer")

# Conditional routing from reviewer
builder.add_conditional_edges(
    "reviewer",
    should_continue,
    {
        "coder": "coder",
        "planner": "planner",
        "skills": "skills",
    },
)

# We can also add edges to cots_search if needed, but for now we'll keep it simple
# Maybe coder can decide to go to cots_search?
# For now let's just make it reachable or at least present.

builder.add_edge("skills", END)
builder.add_edge("cots_search", "planner") # Example path back

# T026: Implement Checkpointing
memory = MemorySaver()

graph = builder.compile(checkpointer=memory)
