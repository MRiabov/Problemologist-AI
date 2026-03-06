import structlog
from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import END, START, StateGraph
from langgraph.types import Command

from controller.agent.config import settings as agent_settings
from controller.agent.context_usage import (
    estimate_text_tokens,
    update_episode_context_usage,
)
from controller.agent.execution_limits import evaluate_agent_hard_fail
from controller.agent.node_entry_validation import (
    ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK,
    NodeEntryValidationError,
    ValidationGraph,
    build_engineer_node_contracts,
    evaluate_node_entry_contract,
    integration_mode_enabled,
    reviewer_handover_custom_check_from_session_id,
)
from controller.clients.worker import WorkerClient
from controller.config.settings import settings as controller_settings
from controller.graph.steerability_node import check_steering, steerability_node
from shared.enums import AgentName
from shared.observability.events import emit_event
from shared.observability.schemas import NodeEntryValidationFailedEvent

from .nodes.coder import coder_node
from .nodes.cots_search import cots_search_node
from .nodes.electronics_engineer import electronics_engineer_node
from .nodes.electronics_planner import electronics_planner_node
from .nodes.electronics_reviewer import electronics_reviewer_node
from .nodes.execution_reviewer import engineer_execution_reviewer_node
from .nodes.plan_reviewer import engineer_plan_reviewer_node
from .nodes.planner import planner_node
from .nodes.skills import skills_node
from .nodes.summarizer import summarizer_node
from .state import AgentState, AgentStatus

logger = structlog.get_logger(__name__)

ENGINEER_NODE_CONTRACTS = build_engineer_node_contracts()


async def _artifact_exists_for_state(state: AgentState, artifact_path: str) -> bool:
    session_id = (state.session_id or "").strip()
    if not session_id:
        logger.error(
            "node_entry_validation_session_missing",
            artifact_path=artifact_path,
            target_node=getattr(state, "current_step", ""),
        )
        return False

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=session_id,
    )
    try:
        return await client.exists(artifact_path)
    except Exception as exc:
        logger.error(
            "node_entry_validation_artifact_lookup_failed",
            artifact_path=artifact_path,
            session_id=session_id,
            error=str(exc),
        )
        return False
    finally:
        await client.aclose()


def _format_entry_errors(errors: list[NodeEntryValidationError]) -> str:
    return "; ".join(f"{error.code}: {error.message}" for error in errors)


def _build_entry_rejection_feedback(
    *,
    target_node: AgentName,
    result,
) -> tuple[str, str]:
    action = (
        f"reroute={result.reroute_target.value}"
        if result.reroute_target
        else f"disposition={result.disposition.value}"
    )
    detail = _format_entry_errors(result.errors)
    feedback = (
        f"ENTRY_VALIDATION_FAILED[{result.reason_code}] "
        f"target={target_node.value} {action} | {detail}"
    )
    journal_entry = (
        f"[Entry Validation] target={target_node.value} "
        f"disposition={result.disposition.value} reason={result.reason_code} "
        f"errors={detail}"
    )
    return feedback, journal_entry


async def _evaluate_engineer_node_entry(target_node: AgentName, state: AgentState):
    custom_checks = {
        ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=getattr(state, "session_id", None),
                reviewer_label="Execution",
            )
        )
    }
    contract = ENGINEER_NODE_CONTRACTS[target_node]
    return await evaluate_node_entry_contract(
        contract=contract,
        state=state,
        artifact_exists=lambda path: _artifact_exists_for_state(state, path),
        graph=ValidationGraph.ENGINEER,
        custom_checks=custom_checks,
    )


def _guarded_node(target_node: AgentName, node_callable):
    async def _run(state: AgentState):
        # Entry validation is intentionally scoped to first-class graph transitions.
        # Tool-invoked helper subagents are out of scope for this contract.
        validation = await _evaluate_engineer_node_entry(target_node, state)
        if validation.ok:
            state.entry_validation_rejected = False
            state.entry_validation_terminal = False
            state.entry_validation_reason_code = None
            state.entry_validation_target_node = None
            state.entry_validation_disposition = None
            state.entry_validation_reroute_target = None
            state.entry_validation_errors = []
            state.entry_validation_trace_emitted = False
            return await node_callable(state)

        feedback, journal_line = _build_entry_rejection_feedback(
            target_node=target_node,
            result=validation,
        )
        serialized_errors = [
            error.model_dump(mode="json") for error in validation.errors
        ]
        current_journal = state.journal or ""
        update = {
            "feedback": feedback,
            "journal": (current_journal + "\n" + journal_line).strip(),
            "entry_validation_rejected": True,
            "entry_validation_reason_code": validation.reason_code,
            "entry_validation_target_node": target_node.value,
            "entry_validation_disposition": validation.disposition.value,
            "entry_validation_reroute_target": (
                validation.reroute_target.value if validation.reroute_target else None
            ),
            "entry_validation_errors": serialized_errors,
            "entry_validation_trace_emitted": False,
        }

        emit_event(
            NodeEntryValidationFailedEvent(
                episode_id=state.episode_id or None,
                user_session_id=state.session_id or None,
                node=target_node.value,
                disposition=validation.disposition.value,
                reason_code=validation.reason_code,
                errors=serialized_errors,
                reroute_target=(
                    validation.reroute_target.value
                    if validation.reroute_target
                    else None
                ),
            )
        )
        logger.error(
            "node_entry_validation_rejected",
            episode_id=state.episode_id,
            target_node=target_node.value,
            disposition=validation.disposition.value,
            reason_code=validation.reason_code,
            reroute_target=(
                validation.reroute_target.value if validation.reroute_target else None
            ),
            integration_mode=integration_mode_enabled(),
            errors=serialized_errors,
        )

        if (
            validation.disposition.value == "reroute_previous"
            and validation.reroute_target is not None
        ):
            update["entry_validation_terminal"] = False
            return Command(goto=validation.reroute_target, update=update)

        update["status"] = AgentStatus.FAILED
        update["entry_validation_terminal"] = True
        return Command(goto=END, update=update)

    return _run


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

    hard_fail = await evaluate_agent_hard_fail(
        agent_name=AgentName.ENGINEER_CODER,
        episode_id=state.episode_id,
        turn_count=state.turn_count,
    )
    if hard_fail.should_fail:
        state.status = AgentStatus.FAILED
        state.feedback = hard_fail.message or "Agent hard-fail limit reached."
        state.journal = (
            state.journal + "\n[Hard Fail] " + (hard_fail.message or "quota reached")
        ).strip()
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


async def should_continue_after_plan_review(state: AgentState) -> str:
    """Route after plan reviewer. Approved plans must proceed to implementation."""
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

    if (
        estimate_text_tokens(state.journal)
        > agent_settings.context_compaction_threshold_tokens
    ):
        return AgentName.JOURNALLING_AGENT

    if state.status == AgentStatus.APPROVED:
        return AgentName.ENGINEER_CODER

    hard_fail = await evaluate_agent_hard_fail(
        agent_name=AgentName.ENGINEER_CODER,
        episode_id=state.episode_id,
        turn_count=state.turn_count,
    )
    if hard_fail.should_fail:
        state.status = AgentStatus.FAILED
        state.feedback = hard_fail.message or "Agent hard-fail limit reached."
        state.journal = (
            state.journal + "\n[Hard Fail] " + (hard_fail.message or "quota reached")
        ).strip()
        return AgentName.SKILL_AGENT

    if state.status == AgentStatus.FAILED:
        return AgentName.SKILL_AGENT

    if state.iteration < 5:
        if state.status == AgentStatus.PLAN_REJECTED:
            return AgentName.ENGINEER_PLANNER
        return AgentName.ENGINEER_CODER

    return AgentName.SKILL_AGENT


# Initialize the StateGraph with our AgentState
builder = StateGraph(AgentState)

# Add nodes
builder.add_node(
    AgentName.ENGINEER_PLANNER,
    _guarded_node(AgentName.ENGINEER_PLANNER, planner_node),
)
builder.add_node(
    AgentName.ELECTRONICS_PLANNER,
    _guarded_node(AgentName.ELECTRONICS_PLANNER, electronics_planner_node),
)
builder.add_node(
    AgentName.ENGINEER_PLAN_REVIEWER,
    _guarded_node(AgentName.ENGINEER_PLAN_REVIEWER, engineer_plan_reviewer_node),
)
builder.add_node(
    AgentName.ENGINEER_CODER,
    _guarded_node(AgentName.ENGINEER_CODER, coder_node),
)
builder.add_node(
    AgentName.ELECTRONICS_ENGINEER,
    _guarded_node(AgentName.ELECTRONICS_ENGINEER, electronics_engineer_node),
)
builder.add_node(
    AgentName.ELECTRONICS_REVIEWER,
    _guarded_node(AgentName.ELECTRONICS_REVIEWER, electronics_reviewer_node),
)
builder.add_node(
    AgentName.ENGINEER_EXECUTION_REVIEWER,
    _guarded_node(
        AgentName.ENGINEER_EXECUTION_REVIEWER, engineer_execution_reviewer_node
    ),
)
builder.add_node(
    AgentName.COTS_SEARCH,
    _guarded_node(AgentName.COTS_SEARCH, cots_search_node),
)
builder.add_node(
    AgentName.SKILL_AGENT,
    _guarded_node(AgentName.SKILL_AGENT, skills_node),
)
builder.add_node(
    AgentName.JOURNALLING_AGENT,
    _guarded_node(AgentName.JOURNALLING_AGENT, summarizer_node),
)
builder.add_node(AgentName.STEER, _guarded_node(AgentName.STEER, steerability_node))

# Set the entry point and edges
builder.add_edge(START, AgentName.ENGINEER_PLANNER)
builder.add_edge(AgentName.ENGINEER_PLANNER, AgentName.ELECTRONICS_PLANNER)
builder.add_edge(AgentName.ELECTRONICS_PLANNER, AgentName.ENGINEER_PLAN_REVIEWER)

builder.add_conditional_edges(
    AgentName.ENGINEER_PLAN_REVIEWER,
    should_continue_after_plan_review,
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
    {AgentName.STEER: AgentName.STEER, "next": AgentName.ENGINEER_EXECUTION_REVIEWER},
)

# Conditional routing from execution reviewer
builder.add_conditional_edges(
    AgentName.ENGINEER_EXECUTION_REVIEWER,
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


def _build_single_node_graph(node_name: AgentName, node_callable):
    single = StateGraph(AgentState)
    single.add_node(node_name, _guarded_node(node_name, node_callable))
    single.add_edge(START, node_name)
    single.add_edge(node_name, END)
    return single.compile(checkpointer=MemorySaver())


engineer_planner_graph = _build_single_node_graph(
    AgentName.ENGINEER_PLANNER, planner_node
)
electronics_planner_graph = _build_single_node_graph(
    AgentName.ELECTRONICS_PLANNER, electronics_planner_node
)
