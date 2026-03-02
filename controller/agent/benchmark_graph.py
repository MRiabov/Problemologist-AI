import structlog
from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph

from controller.config.settings import settings
from controller.graph.steerability_node import check_steering, steerability_node

from .nodes.benchmark_coder import benchmark_coder_node
from .nodes.benchmark_planner import benchmark_planner_node
from .nodes.benchmark_plan_reviewer import benchmark_plan_reviewer_node
from .nodes.execution_reviewer import execution_reviewer_node
from .nodes.plan_reviewer import plan_reviewer_node
from .nodes.skills import skills_node
from .nodes.summarizer import summarizer_node
from .state import AgentState, AgentStatus

logger = structlog.get_logger(__name__)


async def should_continue(state: AgentState) -> str:
    """Route after reviewer based on approval status."""
    if await check_steering(state) == "steer":
        return "steer"

    if len(state.journal) > 5000:
        return "summarizer"

    if state.turn_count >= settings.max_agent_turns:
        return "skills"

    if state.status == AgentStatus.APPROVED or state.status == AgentStatus.FAILED:
        if state.status == AgentStatus.APPROVED and "- [ ]" in state.todo:
            return "coder"
        return "skills"

    if state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            return "planner"
        return "coder"

    return "skills"


builder = StateGraph(AgentState)

# Add nodes
builder.add_node("planner", benchmark_planner_node)
builder.add_node("plan_reviewer", benchmark_plan_reviewer_node)
builder.add_node("coder", benchmark_coder_node)
builder.add_node("execution_reviewer", execution_reviewer_node)
builder.add_node("skills", skills_node)
builder.add_node("summarizer", summarizer_node)
builder.add_node("steer", steerability_node)

# Set the entry point and edges
builder.add_edge(START, "planner")
builder.add_edge("planner", "plan_reviewer")

builder.add_conditional_edges(
    "plan_reviewer",
    should_continue,
    {
        "coder": "coder",
        "planner": "planner",
        "skills": "skills",
        "steer": "steer",
        "summarizer": "summarizer",
    },
)

builder.add_conditional_edges(
    "coder",
    check_steering,
    {"steer": "steer", "next": "execution_reviewer"},
)

builder.add_conditional_edges(
    "execution_reviewer",
    should_continue,
    {
        "coder": "coder",
        "planner": "planner",
        "skills": "skills",
        "steer": "steer",
        "summarizer": "summarizer",
    },
)

builder.add_edge("steer", "planner")
builder.add_edge("skills", END)
builder.add_edge("summarizer", "planner")

memory = MemorySaver()
benchmark_graph = builder.compile(checkpointer=memory)
