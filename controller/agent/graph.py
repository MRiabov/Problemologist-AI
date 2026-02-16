import structlog
from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph

from controller.config.settings import settings
from controller.graph.steerability_node import steerability_node, check_steering

from .nodes.planner import planner_node
from .nodes.reviewer import reviewer_node
from .nodes.coder import coder_node
from .nodes.electronics_engineer import electronics_engineer_node
from .nodes.electronics_reviewer import electronics_reviewer_node
from .nodes.compressor import compressor_node
from .nodes.skills import skills_node
from .nodes.cots_search import cots_search_node
from .state import AgentState, AgentStatus

logger = structlog.get_logger(__name__)


async def should_continue(state: AgentState) -> str:
    """Route after reviewer based on approval status."""
    if await check_steering(state) == "steer":
        return "steer"

    # WP3: Determine intended next node
    intended_next = "skills"
    if state.status == AgentStatus.APPROVED:
        if "- [ ]" in state.todo:
            intended_next = "coder"
    elif state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            intended_next = "planner"
        else:
            intended_next = "coder"

    # WP3: Token Compression Check
    if len(state.messages) > 20 or state.turn_count > settings.max_agent_turns * 0.8:
        state.next_node = intended_next
        return "compressor"

    if state.turn_count >= settings.max_agent_turns:
        return "skills"

    return intended_next


# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node("planner", planner_node)
builder.add_node("coder", coder_node)
builder.add_node("electronics_engineer", electronics_engineer_node)
builder.add_node("electronics_reviewer", electronics_reviewer_node)
builder.add_node("reviewer", reviewer_node)
builder.add_node("cots_search", cots_search_node)
builder.add_node("skills", skills_node)
builder.add_node("compressor", compressor_node)
builder.add_node("steer", steerability_node)

# Set the entry point and edges
builder.add_edge(START, "planner")
builder.add_edge("planner", "coder")

builder.add_conditional_edges(
    "coder",
    check_steering,
    {"steer": "steer", "next": "electronics_engineer"},
)

builder.add_conditional_edges(
    "electronics_engineer",
    check_steering,
    {"steer": "steer", "next": "electronics_reviewer"},
)

builder.add_conditional_edges(
    "electronics_reviewer",
    check_steering,
    {"steer": "steer", "next": "reviewer"},
)

# Conditional routing from reviewer
builder.add_conditional_edges(
    "reviewer",
    should_continue,
    {
        "coder": "coder",
        "planner": "planner",
        "skills": "skills",
        "steer": "steer",
        "compressor": "compressor",
    },
)

def route_after_compression(state: AgentState) -> str:
    return state.next_node or "reviewer"

builder.add_conditional_edges(
    "compressor",
    route_after_compression,
    {"coder": "coder", "planner": "planner", "skills": "skills", "reviewer": "reviewer"},
)

builder.add_edge("steer", "planner")

# We can also add edges to cots_search if needed, but for now we'll keep it simple
# Maybe coder can decide to go to cots_search?
# For now let's just make it reachable or at least present.

builder.add_edge("skills", END)
builder.add_edge("cots_search", "planner")  # Example path back

# T026: Implement Checkpointing
memory = MemorySaver()

graph = builder.compile(checkpointer=memory)
