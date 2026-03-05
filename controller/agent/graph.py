import structlog
from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph

from controller.agent.config import settings as agent_settings
from controller.agent.context_usage import (
    estimate_text_tokens,
    update_episode_context_usage,
)
from controller.config.settings import settings
from controller.graph.steerability_node import check_steering, steerability_node
from shared.enums import AgentName

from .nodes.coder import coder_node
from .nodes.cots_search import cots_search_node
from .nodes.electronics_engineer import electronics_engineer_node
from .nodes.electronics_planner import electronics_planner_node
from .nodes.electronics_reviewer import electronics_reviewer_node
from .nodes.execution_reviewer import execution_reviewer_node
from .nodes.plan_reviewer import plan_reviewer_node
from .nodes.planner import planner_node
from .nodes.skills import skills_node
from .nodes.summarizer import summarizer_node
from .state import AgentState, AgentStatus

logger = structlog.get_logger(__name__)


async def should_continue(state: AgentState) -> str:
    """Route after reviewer based on approval status."""
    if await check_steering(state) == AgentName.STEER:
        return AgentName.STEER

    if state.episode_id:
        try:
            threshold = agent_settings.context_compaction_threshold_tokens
            journal_tokens = estimate_text_tokens(state.journal)
            await update_episode_context_usage(
                episode_id=state.episode_id,
                used_tokens=journal_tokens,
                max_tokens=threshold,
            )
        except Exception as exc:
            logger.warning(
                "context_usage_event_emit_failed",
                error=str(exc),
                episode_id=state.episode_id,
            )

    # Check for summarization need if journal is long
    if (
        estimate_text_tokens(state.journal)
        > agent_settings.context_compaction_threshold_tokens
    ):
        return AgentName.JOURNALLING_AGENT

    if state.turn_count >= settings.max_agent_turns:
        return AgentName.SKILL_AGENT

    if state.status == AgentStatus.APPROVED or state.status == AgentStatus.FAILED:
        # T010: Check if there are more steps in TODO before finishing
        if state.status == AgentStatus.APPROVED and "- [ ]" in state.todo:
            logger.info("step_approved_continuing_to_next", todo=state.todo)
            return AgentName.ENGINEER_CODER
        return AgentName.SKILL_AGENT

    # If rejected and we haven't looped too many times
    if state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            return AgentName.ENGINEER_PLANNER
        return AgentName.ENGINEER_CODER

    return AgentName.SKILL_AGENT


# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node(AgentName.ENGINEER_PLANNER, planner_node)
builder.add_node(AgentName.ELECTRONICS_PLANNER, electronics_planner_node)
builder.add_node(AgentName.ENGINEER_REVIEWER, plan_reviewer_node)
builder.add_node(AgentName.ENGINEER_CODER, coder_node)
builder.add_node(AgentName.ELECTRONICS_ENGINEER, electronics_engineer_node)
builder.add_node(AgentName.ELECTRONICS_REVIEWER, electronics_reviewer_node)
builder.add_node(AgentName.EXECUTION_REVIEWER, execution_reviewer_node)
builder.add_node(AgentName.COTS_SEARCH, cots_search_node)
builder.add_node(AgentName.SKILL_AGENT, skills_node)
builder.add_node(AgentName.JOURNALLING_AGENT, summarizer_node)
builder.add_node(AgentName.STEER, steerability_node)

# Set the entry point and edges
builder.add_edge(START, AgentName.ENGINEER_PLANNER)
builder.add_edge(AgentName.ENGINEER_PLANNER, AgentName.ELECTRONICS_PLANNER)
builder.add_edge(AgentName.ELECTRONICS_PLANNER, AgentName.ENGINEER_REVIEWER)

builder.add_conditional_edges(
    AgentName.ENGINEER_REVIEWER,
    should_continue,
    {
        AgentName.ENGINEER_CODER: AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_PLANNER: AgentName.ENGINEER_PLANNER,
        AgentName.SKILL_AGENT: AgentName.SKILL_AGENT,
        AgentName.STEER: AgentName.STEER,
        AgentName.JOURNALLING_AGENT: AgentName.JOURNALLING_AGENT,
    },
)

builder.add_conditional_edges(
    AgentName.ENGINEER_CODER,
    check_steering,
    {AgentName.STEER: AgentName.STEER, "next": AgentName.ELECTRONICS_ENGINEER},
)

builder.add_conditional_edges(
    AgentName.ELECTRONICS_ENGINEER,
    check_steering,
    {AgentName.STEER: AgentName.STEER, "next": AgentName.ELECTRONICS_REVIEWER},
)

builder.add_conditional_edges(
    AgentName.ELECTRONICS_REVIEWER,
    check_steering,
    {AgentName.STEER: AgentName.STEER, "next": AgentName.EXECUTION_REVIEWER},
)

# Conditional routing from execution reviewer
builder.add_conditional_edges(
    AgentName.EXECUTION_REVIEWER,
    should_continue,
    {
        AgentName.ENGINEER_CODER: AgentName.ENGINEER_CODER,
        AgentName.ENGINEER_PLANNER: AgentName.ENGINEER_PLANNER,
        AgentName.SKILL_AGENT: AgentName.SKILL_AGENT,
        AgentName.STEER: AgentName.STEER,
        AgentName.JOURNALLING_AGENT: AgentName.JOURNALLING_AGENT,
    },
)

builder.add_edge(AgentName.STEER, AgentName.ENGINEER_PLANNER)

builder.add_edge(AgentName.SKILL_AGENT, END)
builder.add_edge(AgentName.JOURNALLING_AGENT, AgentName.ENGINEER_PLANNER)
builder.add_edge(AgentName.COTS_SEARCH, AgentName.ENGINEER_PLANNER)

# T026: Implement Checkpointing
memory = MemorySaver()

graph = builder.compile(checkpointer=memory)
