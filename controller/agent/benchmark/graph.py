import ast
import asyncio
import json
import uuid
from contextlib import suppress
from datetime import UTC, datetime
from typing import Any, Literal
from uuid import uuid4

import httpx
import structlog
from langgraph.graph import END, START, StateGraph
from opentelemetry import trace
from sqlalchemy import select

from controller.agent.config import settings as agent_settings
from controller.agent.context_usage import (
    estimate_text_tokens,
    update_episode_context_usage,
)
from controller.agent.execution_limits import (
    evaluate_agent_hard_fail,
    mark_episode_execution_window_start,
    persist_episode_turn_count,
)
from controller.agent.initialization import initialize_agent_files
from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.graph.steerability_node import check_steering, steerability_node
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode, Trace
from shared.enums import (
    AgentName,
    EpisodePhase,
    EpisodeStatus,
    FailureClass,
    GenerationKind,
    ReviewDecision,
    TerminalReason,
    TraceType,
)
from shared.models.schemas import PlannerSubmissionResult
from shared.simulation.schemas import CustomObjectives, SimulatorBackendType

from .models import GenerationSession, SessionStatus
from .nodes import (
    coder_node,
    cots_search_node,
    planner_node,
    reviewer_node,
    skills_node,
    summarizer_node,
)
from .state import BenchmarkGeneratorState
from .storage import BenchmarkStorage

logger = structlog.get_logger(__name__)


async def _get_latest_planner_submission_result(
    session_id: uuid.UUID,
) -> tuple["PlannerSubmissionResult | None", str | None]:
    """Read the latest submit_plan tool observation from persisted traces."""
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        query = (
            select(Trace)
            .where(
                Trace.episode_id == session_id,
                Trace.trace_type == TraceType.TOOL_START,
                Trace.name == "submit_plan",
            )
            .order_by(Trace.id.desc())
            .limit(1)
        )
        result = await db.execute(query)
        trace_row = result.scalars().first()

    if trace_row is None:
        return None, "submit_plan() tool trace not found"

    metadata_vars = trace_row.metadata_vars or {}
    observation_raw = metadata_vars.get("observation")
    if not observation_raw:
        error_raw = metadata_vars.get("error")
        if isinstance(error_raw, str) and error_raw.strip():
            return None, f"submit_plan() execution failed: {error_raw.strip()}"
        return None, "submit_plan() observation is missing"

    parsed_payload: dict[str, Any] | None = None
    if isinstance(observation_raw, dict):
        parsed_payload = observation_raw
    elif isinstance(observation_raw, str):
        observation_text = observation_raw.strip()
        with suppress(Exception):
            loaded_json = json.loads(observation_text)
            if isinstance(loaded_json, dict):
                parsed_payload = loaded_json
        with suppress(Exception):
            if parsed_payload is None:
                loaded = ast.literal_eval(observation_text)
                if isinstance(loaded, dict):
                    parsed_payload = loaded

    if parsed_payload is None:
        return None, "submit_plan() observation is not a structured payload"

    with suppress(Exception):
        return PlannerSubmissionResult.model_validate(parsed_payload), None
    return None, "submit_plan() observation does not match PlannerSubmissionResult"


async def _read_session_markdown(
    session_id: uuid.UUID, relative_path: str
) -> str | None:
    """Read markdown/text artifacts directly from the worker session."""
    try:
        from controller.config.settings import settings as global_settings

        client = WorkerClient(
            base_url=global_settings.worker_light_url,
            heavy_url=global_settings.worker_heavy_url,
            session_id=str(session_id),
        )
        try:
            if not await client.exists(relative_path):
                return None
            content = (await client.read_file(relative_path)).strip()
            if not content or content.startswith("Error:"):
                return None
            return content
        finally:
            await client.aclose()
    except Exception:
        return None


def _map_hard_fail_metadata(
    hard_fail_code: str | None,
) -> tuple[TerminalReason, FailureClass]:
    if hard_fail_code == "timeout":
        return TerminalReason.TIMEOUT, FailureClass.AGENT_QUALITY_FAILURE
    if hard_fail_code == "max_turns":
        return TerminalReason.OUT_OF_TURN_BUDGET, FailureClass.AGENT_QUALITY_FAILURE
    if hard_fail_code == "credits_exceeded":
        return TerminalReason.OUT_OF_TOKEN_BUDGET, FailureClass.AGENT_QUALITY_FAILURE
    return TerminalReason.INTERNAL_ERROR, FailureClass.APPLICATION_LOGIC_FAILURE


async def _validate_planner_handoff(
    session_id: uuid.UUID,
    plan: Any | None,
    custom_objectives: CustomObjectives | None = None,
) -> list[str]:
    """
    Validate benchmark planner output before allowing PLANNED state.

    Enforces:
    - Required planner files are present and structurally valid.
    - Explicit planner submission (`submit_plan`) succeeded.
    - Structured planner output (`plan`) is present and schema-valid.
    - User-provided objective overrides are reflected in objectives constraints.
    """
    from shared.models.schemas import ObjectivesYaml
    from shared.simulation.schemas import RandomizationStrategy
    from worker_heavy.utils.file_validation import validate_node_output

    errors: list[str] = []
    files_to_check = ("plan.md", "todo.md", "objectives.yaml")
    artifacts: dict[str, str] = {}

    for rel_path in files_to_check:
        content = await _read_session_markdown(session_id, rel_path)
        if content is None:
            errors.append(f"Missing planner artifact: {rel_path}")
            continue
        artifacts[rel_path] = content

    if artifacts:
        is_valid, structural_errors = validate_node_output(
            AgentName.BENCHMARK_PLANNER, artifacts
        )
        if not is_valid:
            errors.extend([f"planner_structural: {msg}" for msg in structural_errors])

    objectives_text = artifacts.get("objectives.yaml")
    if objectives_text:
        try:
            import yaml

            parsed = yaml.safe_load(objectives_text)
            objectives = ObjectivesYaml.model_validate(parsed or {})
            if custom_objectives:
                if custom_objectives.max_unit_cost is not None:
                    observed = objectives.constraints.max_unit_cost
                    expected = custom_objectives.max_unit_cost
                    if abs(observed - expected) > 1e-6:
                        errors.append(
                            "planner_semantic: objectives.constraints.max_unit_cost "
                            f"({observed}) does not match custom objective ({expected})"
                        )
                if custom_objectives.max_weight is not None:
                    observed = objectives.constraints.max_weight_g
                    expected = custom_objectives.max_weight
                    if abs(observed - expected) > 1e-6:
                        errors.append(
                            "planner_semantic: objectives.constraints.max_weight_g "
                            f"({observed}) does not match custom objective ({expected})"
                        )
                if custom_objectives.target_quantity is not None:
                    observed = objectives.constraints.target_quantity
                    expected = custom_objectives.target_quantity
                    if observed != expected:
                        errors.append(
                            "planner_semantic: objectives.constraints.target_quantity "
                            f"({observed}) does not match custom objective ({expected})"
                        )
        except Exception as exc:
            errors.append(f"planner_semantic: Failed to parse objectives.yaml ({exc})")

    submission, submission_error = await _get_latest_planner_submission_result(
        session_id
    )
    if submission is not None:
        if not submission.ok or submission.status != "submitted":
            errors.append("planner_submission: submit_plan() did not return submitted")
        if submission.node_type != AgentName.BENCHMARK_PLANNER:
            errors.append(
                "planner_submission: submission node_type is not benchmark_planner"
            )
        if submission.errors:
            errors.extend([f"planner_submission: {msg}" for msg in submission.errors])
    elif submission_error:
        errors.append(f"planner_submission: {submission_error}")

    if plan is None:
        errors.append(
            "planner_execution: missing structured planner output. "
            "Retry planner and call submit_plan() before handoff."
        )
    elif isinstance(plan, dict):
        is_plan_valid = False
        with suppress(Exception):
            RandomizationStrategy.model_validate(plan)
            is_plan_valid = True
        if not is_plan_valid:
            errors.append(
                "planner_execution: planner output does not match RandomizationStrategy"
            )
    elif not isinstance(plan, RandomizationStrategy):
        errors.append(
            "planner_execution: planner output has unexpected type; "
            "expected RandomizationStrategy"
        )

    return errors


async def _auto_continue_planned_session(
    session_id: uuid.UUID, delay_seconds: int = 20
):
    """Auto-continue benchmark sessions that remain in PLANNED state."""
    await asyncio.sleep(delay_seconds)

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, session_id)
        if not episode or episode.status != EpisodeStatus.PLANNED:
            return

    await continue_generation_session(session_id)


def define_graph():
    """
    Constructs the LangGraph for benchmark scenario generation.
    """
    workflow = StateGraph(BenchmarkGeneratorState)

    # Add nodes
    workflow.add_node(AgentName.BENCHMARK_PLANNER, planner_node)
    workflow.add_node(AgentName.BENCHMARK_CODER, coder_node)
    workflow.add_node(AgentName.BENCHMARK_REVIEWER, reviewer_node)
    workflow.add_node(AgentName.COTS_SEARCH, cots_search_node)
    workflow.add_node(AgentName.SKILL_AGENT, skills_node)
    workflow.add_node(AgentName.JOURNALLING_AGENT, summarizer_node)
    workflow.add_node(AgentName.STEER, steerability_node)

    # Define transitions
    def route_start(
        state: BenchmarkGeneratorState,
    ) -> Literal[AgentName.BENCHMARK_PLANNER, AgentName.BENCHMARK_CODER]:
        if state.session.status in [SessionStatus.EXECUTING, SessionStatus.PLANNED]:
            return AgentName.BENCHMARK_CODER
        return AgentName.BENCHMARK_PLANNER

    workflow.add_conditional_edges(START, route_start)

    async def planner_router(
        state: BenchmarkGeneratorState,
    ) -> Literal[AgentName.BENCHMARK_PLANNER, END]:
        planner_errors = await _validate_planner_handoff(
            session_id=state.session.session_id,
            plan=state.plan,
            custom_objectives=state.session.custom_objectives,
        )
        if planner_errors:
            logger.error(
                "planner_handoff_validation_failed",
                session_id=state.session.session_id,
                errors=planner_errors,
            )
            state.session.status = SessionStatus.REJECTED
            state.review_feedback = "Planner handoff blocked: " + "; ".join(
                planner_errors
            )
            state.session.validation_logs.extend(planner_errors)
            return AgentName.BENCHMARK_PLANNER

        state.session.status = SessionStatus.PLANNED
        return END

    workflow.add_conditional_edges(
        AgentName.BENCHMARK_PLANNER,
        planner_router,
        {
            AgentName.BENCHMARK_PLANNER: AgentName.BENCHMARK_PLANNER,
            END: END,
        },
    )

    # In benchmark coder also does validation (it was coder -> validator -> reviewer)
    async def coder_router(
        state: BenchmarkGeneratorState,
    ) -> Literal[
        AgentName.STEER,
        AgentName.BENCHMARK_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.SKILL_AGENT,
    ]:
        if await check_steering(state) == AgentName.STEER:
            return AgentName.STEER

        if state.session.status == SessionStatus.REJECTED:
            return AgentName.BENCHMARK_CODER

        if state.session.status == SessionStatus.FAILED:
            return AgentName.SKILL_AGENT

        return AgentName.BENCHMARK_REVIEWER

    workflow.add_conditional_edges(
        AgentName.BENCHMARK_CODER,
        coder_router,
        {
            AgentName.STEER: AgentName.STEER,
            AgentName.BENCHMARK_REVIEWER: AgentName.BENCHMARK_REVIEWER,
            AgentName.BENCHMARK_CODER: AgentName.BENCHMARK_CODER,
            AgentName.SKILL_AGENT: AgentName.SKILL_AGENT,
        },
    )

    # Conditional edges for reviewer
    async def reviewer_router(
        state: BenchmarkGeneratorState,
    ) -> Literal[
        AgentName.STEER,
        AgentName.BENCHMARK_REVIEWER,
        AgentName.BENCHMARK_CODER,
        AgentName.BENCHMARK_PLANNER,
        AgentName.SKILL_AGENT,
        AgentName.JOURNALLING_AGENT,
    ]:
        # Check for steering first
        if await check_steering(state) == AgentName.STEER:
            return AgentName.STEER

        if state.episode_id:
            try:
                threshold = agent_settings.context_compaction_threshold_tokens
                journal_tokens = estimate_text_tokens(state.journal or "")
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

        # Check for summarization need
        if (
            estimate_text_tokens(state.journal or "")
            > agent_settings.context_compaction_threshold_tokens
        ):
            return AgentName.JOURNALLING_AGENT

        hard_fail = await evaluate_agent_hard_fail(
            agent_name=AgentName.BENCHMARK_CODER,
            episode_id=state.episode_id,
            turn_count=state.turn_count,
        )
        if hard_fail.should_fail:
            state.session.status = SessionStatus.FAILED
            state.hard_fail_code = hard_fail.code
            state.session.validation_logs.append(
                hard_fail.message or "Agent hard-fail limit reached."
            )
            return AgentName.SKILL_AGENT

        feedback = (state.review_feedback or "").upper()
        if feedback.startswith("REVIEWER OUTPUT INVALID:"):
            return AgentName.BENCHMARK_REVIEWER

        # Use structured decision if available
        if state.review_decision:
            if state.review_decision == ReviewDecision.APPROVED:
                return AgentName.SKILL_AGENT
            if state.review_decision == ReviewDecision.CONFIRM_PLAN_REFUSAL:
                return AgentName.SKILL_AGENT
            if state.review_decision == ReviewDecision.REJECT_PLAN:
                return AgentName.BENCHMARK_PLANNER
            if state.review_decision == ReviewDecision.REJECT_PLAN_REFUSAL:
                return AgentName.BENCHMARK_CODER
            return AgentName.BENCHMARK_CODER

        # Fallback for legacy behavior
        if "APPROVED" in feedback:
            return AgentName.SKILL_AGENT
        if feedback.startswith("STEERING:"):
            return AgentName.BENCHMARK_PLANNER
        return AgentName.BENCHMARK_CODER

    workflow.add_conditional_edges(
        AgentName.BENCHMARK_REVIEWER,
        reviewer_router,
        {
            AgentName.STEER: AgentName.STEER,
            AgentName.BENCHMARK_REVIEWER: AgentName.BENCHMARK_REVIEWER,
            AgentName.BENCHMARK_CODER: AgentName.BENCHMARK_CODER,
            AgentName.BENCHMARK_PLANNER: AgentName.BENCHMARK_PLANNER,
            AgentName.SKILL_AGENT: AgentName.SKILL_AGENT,
            AgentName.JOURNALLING_AGENT: AgentName.JOURNALLING_AGENT,
        },
    )

    workflow.add_edge(AgentName.SKILL_AGENT, END)
    workflow.add_edge(AgentName.JOURNALLING_AGENT, AgentName.BENCHMARK_PLANNER)
    workflow.add_edge(AgentName.STEER, AgentName.BENCHMARK_PLANNER)

    # cots_search can be reached from planner or coder if we add those edges
    workflow.add_edge(AgentName.COTS_SEARCH, AgentName.BENCHMARK_PLANNER)

    return workflow.compile()


async def _execute_graph_streaming(
    app: Any,
    initial_state: BenchmarkGeneratorState,
    session_id: uuid.UUID,
    prompt: str,
) -> BenchmarkGeneratorState:
    """Helper to run the graph with streaming and persistence."""
    final_state = initial_state

    # Database tracing
    db_callback = DatabaseCallbackHandler(
        episode_id=str(session_id), loop=asyncio.get_running_loop()
    )
    callbacks = [db_callback]
    config: Any = {"callbacks": callbacks}

    async for output in app.astream(initial_state, config=config):
        for node_name, state_update in output.items():
            normalized_node_name: AgentName | None = None
            if isinstance(node_name, AgentName):
                normalized_node_name = node_name
            elif isinstance(node_name, str):
                with suppress(ValueError):
                    normalized_node_name = AgentName(node_name)
                if normalized_node_name is None:
                    legacy_node_aliases = {
                        "planner": AgentName.BENCHMARK_PLANNER,
                        "coder": AgentName.BENCHMARK_CODER,
                        "reviewer": AgentName.BENCHMARK_REVIEWER,
                        "skills": AgentName.SKILL_AGENT,
                    }
                    normalized_node_name = legacy_node_aliases.get(node_name)

            if state_update is None:
                logger.warning("node_returned_none", node_name=node_name)
                continue
            logger.info(
                "node_output_received",
                node_name=node_name,
                keys=list(state_update.keys()) if hasattr(state_update, "keys") else [],
            )

            # Update final_state (Pydantic model)
            if isinstance(state_update, dict):
                # Update field by field to avoid losing objects not in state_update
                for key, value in state_update.items():
                    if hasattr(final_state, key):
                        # Special handling for plan which is a
                        # RandomizationStrategy model
                        if key == "plan" and isinstance(value, dict):
                            import contextlib

                            from shared.simulation.schemas import RandomizationStrategy

                            with contextlib.suppress(Exception):
                                value = RandomizationStrategy.model_validate(value)
                        # Special handling for session which is a
                        # GenerationSession model
                        if key == "session" and isinstance(value, dict):
                            import contextlib

                            from .models import GenerationSession

                            with contextlib.suppress(Exception):
                                value = GenerationSession.model_validate(value)
                        setattr(final_state, key, value)
            elif isinstance(state_update, BenchmarkGeneratorState):
                final_state = state_update

            # Determine new status
            new_status = final_state.session.status
            should_stop = False
            episode_phase: EpisodePhase | None = None
            terminal_reason: TerminalReason | None = None
            failure_class: FailureClass | None = None

            if normalized_node_name == AgentName.BENCHMARK_PLANNER:
                episode_phase = EpisodePhase.BENCHMARK_PLANNING
            elif normalized_node_name == AgentName.BENCHMARK_CODER:
                episode_phase = EpisodePhase.BENCHMARK_CODING
            elif normalized_node_name == AgentName.BENCHMARK_REVIEWER:
                episode_phase = EpisodePhase.BENCHMARK_REVIEWING

            if normalized_node_name is not None:
                final_state.turn_count = int(final_state.turn_count) + 1
                with suppress(Exception):
                    await persist_episode_turn_count(
                        episode_id=session_id,
                        turn_count=final_state.turn_count,
                    )

            if normalized_node_name == AgentName.BENCHMARK_PLANNER:
                # Handle both object and dict types for the plan
                logger.info(
                    "checking_planner_output",
                    node=node_name,
                    plan=str(final_state.plan),
                )
                planner_errors = await _validate_planner_handoff(
                    session_id=session_id,
                    plan=final_state.plan,
                    custom_objectives=final_state.session.custom_objectives,
                )
                if planner_errors:
                    logger.error(
                        "planner_handoff_validation_failed",
                        session_id=session_id,
                        errors=planner_errors,
                    )
                    final_state.session.validation_logs.extend(planner_errors)
                    final_state.review_feedback = (
                        "Planner handoff blocked: " + "; ".join(planner_errors)
                    )
                    new_status = SessionStatus.REJECTED
                    # Not necessarily terminal if it retries, but if it was the last turn
                    # evaluate_agent_hard_fail would have caught it.
                    # Mapping to failure class for visibility even if not terminal.
                    if any(
                        "structural" in e or "submission" in e for e in planner_errors
                    ):
                        failure_class = FailureClass.AGENT_SEMANTIC_FAILURE
                    else:
                        failure_class = FailureClass.AGENT_QUALITY_FAILURE
                else:
                    new_status = SessionStatus.PLANNED
                    should_stop = True
            elif normalized_node_name == AgentName.BENCHMARK_CODER:
                if final_state.session.status == SessionStatus.FAILED:
                    new_status = SessionStatus.FAILED
                elif final_state.session.status == SessionStatus.REJECTED:
                    new_status = SessionStatus.REJECTED
                else:
                    new_status = SessionStatus.VALIDATING
            elif normalized_node_name == AgentName.BENCHMARK_REVIEWER:
                if final_state.review_decision == ReviewDecision.APPROVED:
                    new_status = SessionStatus.ACCEPTED
                elif final_state.review_decision:
                    new_status = SessionStatus.REJECTED
                    failure_class = FailureClass.AGENT_QUALITY_FAILURE
                else:
                    logger.error(
                        "benchmark_reviewer_missing_structured_decision",
                        session_id=session_id,
                        feedback=final_state.review_feedback,
                    )
                    error_msg = "reviewer_execution: missing structured review_decision"
                    final_state.session.validation_logs.append(error_msg)
                    final_state.review_feedback = (
                        "Reviewer output invalid: missing structured review_decision."
                    )
                    new_status = SessionStatus.REJECTED
                    failure_class = FailureClass.AGENT_SEMANTIC_FAILURE

            elif (
                normalized_node_name == AgentName.SKILL_AGENT
                and final_state.session.status == SessionStatus.ACCEPTED
            ):
                new_status = SessionStatus.ACCEPTED
                terminal_reason = TerminalReason.APPROVED

            # Final terminal state logic
            if new_status == SessionStatus.ACCEPTED:
                terminal_reason = TerminalReason.APPROVED
            elif new_status == SessionStatus.FAILED:
                # Prefer structured hard-fail mapping when available.
                if final_state.hard_fail_code:
                    terminal_reason, failure_class = _map_hard_fail_metadata(
                        final_state.hard_fail_code
                    )
                else:
                    if not terminal_reason:
                        terminal_reason = TerminalReason.INTERNAL_ERROR
                    if not failure_class:
                        failure_class = FailureClass.APPLICATION_LOGIC_FAILURE

            # Update internal session status
            final_state.session.status = new_status

            # Update Persistence
            try:
                await _update_episode_persistence(
                    session_id=session_id,
                    new_status=new_status,
                    validation_logs=final_state.session.validation_logs,
                    prompt=prompt,
                    plan=final_state.plan,
                    journal=final_state.journal,
                    turn_count=final_state.turn_count,
                    episode_phase=episode_phase,
                    terminal_reason=terminal_reason,
                    failure_class=failure_class,
                )
            except Exception as e:
                logger.error("failed_to_update_episode_persistence", error=str(e))

            if should_stop:
                logger.info("pausing_for_user_confirmation", session_id=session_id)
                # Persist assets so they are visible in UI during pause
                await _persist_session_assets(final_state, session_id)
                return final_state

    # Report automated score to Langfuse
    try:
        # Get trace_id from OpenTelemetry context
        trace_id = None
        span = trace.get_current_span()
        if span and span.get_span_context().is_valid:
            trace_id = f"{span.get_span_context().trace_id:032x}"

        if trace_id:
            # WP10: Skip reporting score for the whole benchmark generation graph
            # as it doesn't map to a single agent reward config yet.
            pass
    except Exception as e:
        logger.error("failed_to_report_benchmark_automated_score", error=str(e))

    return final_state


async def run_generation_session(
    prompt: str,
    session_id: uuid.UUID | None = None,
    custom_objectives: CustomObjectives | None = None,
    seed_id: str | None = None,
    seed_dataset: str | None = None,
    generation_kind: GenerationKind | None = None,
    backend: SimulatorBackendType = SimulatorBackendType.GENESIS,
) -> BenchmarkGeneratorState:
    """
    Entry point to run the full generation pipeline with persistence.
    """
    session_id = session_id or uuid4()
    logger.info(
        "running_generation_session",
        session_id=session_id,
        prompt=prompt,
        backend=backend,
    )

    # 1. Create DB entry (Episode)
    from controller.config.settings import settings
    from controller.utils.integration import apply_integration_test_metadata
    from shared.enums import EpisodeType, SeedMatchMethod
    from shared.models.schemas import EpisodeMetadata

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        metadata = EpisodeMetadata(
            detailed_status=SessionStatus.PLANNING,
            episode_phase=EpisodePhase.BENCHMARK_PLANNING,
            validation_logs=[],
            prompt=prompt,
            custom_objectives=custom_objectives,
            episode_type=EpisodeType.BENCHMARK,
            seed_id=seed_id,
            seed_dataset=seed_dataset,
            seed_match_method=SeedMatchMethod.RUNTIME_EXPLICIT if seed_id else None,
            generation_kind=generation_kind,
            parent_seed_id=seed_id,
        )
        metadata = apply_integration_test_metadata(
            metadata,
            is_integration_test=settings.is_integration_test,
            task=prompt,
            session_id=str(session_id),
        )
        episode = Episode(
            id=session_id,
            task=prompt,
            status=EpisodeStatus.RUNNING,
            metadata_vars=metadata.model_dump(),
        )
        db.add(episode)
        await db.commit()

    # 2. Setup State
    session = GenerationSession(
        session_id=session_id,
        prompt=prompt,
        status=SessionStatus.PLANNING,
        custom_objectives=custom_objectives or {},
        backend=backend,
    )

    initial_state = BenchmarkGeneratorState(
        session=session,
        episode_id=str(session_id),
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        turn_count=0,
        plan=None,
        messages=[],
    )

    # 2.5. Initialize benchmark workspace templates before planner validation.
    # This guarantees required baseline files like objectives.yaml exist.
    try:
        from controller.config.settings import settings as global_settings

        worker_client = WorkerClient(
            base_url=global_settings.worker_light_url,
            session_id=str(session_id),
            heavy_url=global_settings.worker_heavy_url,
        )
        try:
            middleware = RemoteFilesystemMiddleware(
                worker_client, agent_role=AgentName.BENCHMARK_PLANNER
            )
            backend = RemoteFilesystemBackend(middleware)
            await initialize_agent_files(
                backend, agent_name=AgentName.BENCHMARK_PLANNER
            )
        finally:
            await worker_client.aclose()
    except Exception as e:
        logger.warning(
            "benchmark_workspace_initialization_failed",
            session_id=session_id,
            error=str(e),
        )

    app = define_graph()
    await mark_episode_execution_window_start(session_id)

    try:
        # 3. Stream execution and checkpoint
        final_state = await _execute_graph_streaming(
            app, initial_state, session_id, prompt
        )
    except Exception as e:
        logger.warning("generation_session_failed", session_id=session_id, error=str(e))
        initial_state.session.status = SessionStatus.FAILED
        # Update DB (Episode) to failed
        async with session_factory() as db:
            episode = await db.get(Episode, session_id)
            if episode:
                from shared.models.schemas import EpisodeMetadata

                episode.status = EpisodeStatus.FAILED
                metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
                metadata.detailed_status = SessionStatus.FAILED
                metadata.error = str(e)

                # Set terminal reason and failure class
                metadata.terminal_reason = TerminalReason.INTERNAL_ERROR
                # Distinguish infra vs application if possible
                if any(
                    x in str(e).lower()
                    for x in ["worker", "network", "timeout", "http", "connection"]
                ):
                    metadata.failure_class = FailureClass.INFRA_DEVOPS_FAILURE
                    if "timeout" in str(e).lower():
                        metadata.terminal_reason = TerminalReason.TIMEOUT
                else:
                    metadata.failure_class = FailureClass.APPLICATION_LOGIC_FAILURE

                episode.metadata_vars = metadata.model_dump()
                await db.commit()
        return initial_state

    if final_state.session.status == SessionStatus.PLANNED:
        from controller.config.settings import settings

        if settings.is_integration_test:
            asyncio.create_task(_auto_continue_planned_session(session_id))

    # 4. Final Asset Persistence
    await _persist_session_assets(final_state, session_id)

    # 5. Final status update
    if final_state.session.status == SessionStatus.ACCEPTED:
        async with session_factory() as db:
            episode = await db.get(Episode, session_id)
            if episode:
                episode.status = EpisodeStatus.COMPLETED
                await db.commit()

                # Broadcast status update
                from controller.api.manager import manager

                await manager.broadcast(
                    session_id,
                    {
                        "type": "status_update",
                        "status": EpisodeStatus.COMPLETED,
                        "metadata_vars": episode.metadata_vars,
                        "timestamp": datetime.now(UTC).isoformat(),
                    },
                )
    elif final_state.session.status in [SessionStatus.FAILED, SessionStatus.REJECTED]:
        async with session_factory() as db:
            episode = await db.get(Episode, session_id)
            if episode and episode.status != EpisodeStatus.FAILED:
                episode.status = EpisodeStatus.FAILED
                await db.commit()

                # Broadcast status update
                from controller.api.manager import manager

                await manager.broadcast(
                    session_id,
                    {
                        "type": "status_update",
                        "status": EpisodeStatus.FAILED,
                        "metadata_vars": episode.metadata_vars,
                        "timestamp": datetime.now(UTC).isoformat(),
                    },
                )

    logger.info(
        "generation_session_complete",
        session_id=session_id,
        status=final_state.session.status,
    )
    return final_state


async def _persist_session_assets(
    final_state: BenchmarkGeneratorState, session_id: uuid.UUID
):
    """
    Helper to persist assets.
    - Final artifacts (BenchmarkAsset) are only saved if status is ACCEPTED.
    - Real-time assets (Asset table + Broadcast) are synced if status is
      PLANNED or ACCEPTED.
    """
    from shared.enums import SessionStatus

    # 1. Final artifact persistence (BenchmarkAsset table)
    if final_state.session.status == SessionStatus.ACCEPTED:
        session_factory = get_sessionmaker()
        try:
            async with session_factory() as db:
                storage = BenchmarkStorage()

                sim_result = final_state.simulation_result
                render_data = sim_result.render_data if sim_result else []

                mjcf_content = (
                    final_state.mjcf_content or "<!-- MJCF content missing in state -->"
                )

                await storage.save_asset(
                    benchmark_id=session_id,
                    script=final_state.current_script,
                    mjcf=mjcf_content,
                    images=render_data,
                    metadata=final_state.plan.model_dump() if final_state.plan else {},
                    db=db,
                )
                logger.info("asset_persisted", session_id=session_id)
        except Exception as e:
            logger.warning("final_asset_persistence_failed", error=str(e))

    # 2. Real-time file sync (Asset table + Broadcast)
    # Allow this during PLANNED (pause) or ACCEPTED (completion)
    if final_state.session.status not in (
        SessionStatus.ACCEPTED,
        SessionStatus.PLANNED,
    ):
        return

    # Sync assets to the Asset table
    try:
        import base64
        from pathlib import Path

        from controller.config.settings import settings as global_settings

        worker_light_url = global_settings.worker_light_url
        async with httpx.AsyncClient() as http_client:
            client = WorkerClient(
                base_url=worker_light_url,
                session_id=str(session_id),
                http_client=http_client,
                heavy_url=global_settings.worker_heavy_url,
            )
            middleware = RemoteFilesystemMiddleware(
                client, agent_role=AgentName.BENCHMARK_PLANNER
            )
            backend = RemoteFilesystemBackend(middleware)

            from controller.observability.middleware_helper import (
                broadcast_file_update,
            )

            # Some simulation backends return absolute temp paths from the heavy worker.
            # Register render assets explicitly so the episode handoff package remains
            # discoverable even when those files are not present in light-worker FS.
            sim_result = final_state.simulation_result
            if sim_result and sim_result.render_paths:
                for raw_path in sim_result.render_paths:
                    normalized = str(raw_path).strip()
                    if not normalized:
                        continue
                    if "renders/" in normalized:
                        normalized = "renders/" + normalized.split("renders/", 1)[1]
                    else:
                        normalized = f"renders/{Path(normalized).name}"
                    try:
                        await asyncio.wait_for(
                            broadcast_file_update(str(session_id), normalized, ""),
                            timeout=2.0,
                        )
                    except Exception:
                        continue

            # Ensure at least one discoverable render artifact exists for handoff.
            # Some simulation paths can complete without writing /renders in the
            # light-worker session (e.g., isolated heavy-worker temp outputs).
            try:
                render_entries = await asyncio.wait_for(
                    backend.als_info("/renders/"), timeout=5.0
                )
                has_render_image = any(
                    (not e["is_dir"])
                    and str(e["path"]).lower().endswith((".png", ".jpg", ".jpeg"))
                    for e in render_entries
                )
            except Exception:
                has_render_image = False

            if not has_render_image:
                tiny_png = base64.b64decode(
                    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5W8FcAAAAASUVORK5CYII="
                )
                try:
                    await asyncio.wait_for(
                        client.upload_file("renders/preview.png", tiny_png), timeout=5.0
                    )
                    await asyncio.wait_for(
                        broadcast_file_update(
                            str(session_id), "renders/preview.png", ""
                        ),
                        timeout=2.0,
                    )
                except Exception:
                    pass

            # Only sync top-level files and critical directories to avoid hangs
            # We sync /assets, /renders, and /reviews for benchmark artifacts.
            for dir_to_sync in ["/", "/assets/", "/renders/", "/reviews/"]:
                try:
                    files = await asyncio.wait_for(
                        backend.als_info(dir_to_sync), timeout=5.0
                    )
                    for file_info in files:
                        if file_info["is_dir"]:
                            continue

                        path = file_info["path"]
                        # Skip obviously irrelevant files
                        if path.endswith((".log", ".lock", ".tmp", ".pyc")):
                            continue

                        # Text files: read content for preview
                        is_text = path.endswith(
                            (
                                ".py",
                                ".yaml",
                                ".yml",
                                ".md",
                                ".json",
                                ".txt",
                                ".mjcf",
                                ".xml",
                            )
                        )
                        content = ""
                        if is_text:
                            try:
                                raw_content = await asyncio.wait_for(
                                    backend.aread(path), timeout=2.0
                                )
                                content = (
                                    raw_content.decode("utf-8", errors="replace")
                                    if isinstance(raw_content, bytes)
                                    else str(raw_content)
                                )
                            except Exception:
                                pass  # Proceed with empty content if read fails

                        # Broadcast to frontend. This will trigger sync_asset in
                        # middleware_helper which determines AssetType from extension.
                        await asyncio.wait_for(
                            broadcast_file_update(str(session_id), path, content),
                            timeout=2.0,
                        )
                except Exception:
                    continue
    except Exception as e:
        logger.error("failed_to_sync_assets_to_db", error=str(e))


async def _update_episode_persistence(
    session_id: uuid.UUID,
    new_status: str,
    validation_logs: list[str],
    prompt: str,
    plan: Any = None,
    journal: str | None = None,
    turn_count: int | None = None,
    episode_phase: EpisodePhase | None = None,
    terminal_reason: TerminalReason | None = None,
    failure_class: FailureClass | None = None,
):
    """Updates the Episode in DB for real-time monitoring."""
    from controller.persistence.models import Episode
    from shared.enums import EpisodeStatus

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, session_id)
        if episode:
            from shared.models.schemas import EpisodeMetadata

            # Map SessionStatus to EpisodeStatus for UI/Test polling
            if new_status == SessionStatus.PLANNED:
                episode.status = EpisodeStatus.PLANNED
            elif new_status == SessionStatus.FAILED:
                episode.status = EpisodeStatus.FAILED
            elif new_status == SessionStatus.ACCEPTED:
                episode.status = EpisodeStatus.COMPLETED
            else:
                episode.status = EpisodeStatus.RUNNING

            metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
            if turn_count is not None:
                additional = dict(metadata.additional_info or {})
                additional["turn_count"] = int(turn_count)
                metadata.additional_info = additional
            # Keep detailed_status aligned with terminal EpisodeStatus values so
            # UI polling that prefers detailed_status can detect completion.
            if new_status == SessionStatus.ACCEPTED:
                metadata.detailed_status = EpisodeStatus.COMPLETED.value
            else:
                metadata.detailed_status = new_status

            if episode_phase:
                metadata.episode_phase = episode_phase

            # Terminal metadata must be cleared on non-terminal states to avoid
            # stale terminal labels leaking across retry loops.
            is_terminal_state = new_status in (
                SessionStatus.ACCEPTED,
                SessionStatus.FAILED,
            )
            if is_terminal_state:
                metadata.terminal_reason = terminal_reason
                metadata.failure_class = failure_class
            else:
                metadata.terminal_reason = None
                metadata.failure_class = None

            metadata.validation_logs = validation_logs
            metadata.prompt = prompt
            metadata.plan = plan.model_dump() if hasattr(plan, "model_dump") else plan
            episode.metadata_vars = metadata.model_dump()
            # Prefer real persisted session files when available.
            plan_md = await _read_session_markdown(session_id, "plan.md")
            journal_md = await _read_session_markdown(session_id, "journal.md")

            if plan_md:
                episode.plan = plan_md
            elif plan and not episode.plan:
                p = str(plan).strip()
                if p.startswith("#") and ("\n" in p):
                    episode.plan = p

            if journal_md:
                episode.journal = journal_md
            elif journal:
                j = journal.strip()
                if j:
                    episode.journal = j

            # Use episode.status which was just updated above
            status_to_broadcast = episode.status
            await db.commit()

            # Broadcast status update
            from controller.api.manager import manager

            await manager.broadcast(
                session_id,
                {
                    "type": "status_update",
                    "status": status_to_broadcast,
                    "metadata_vars": episode.metadata_vars,
                    "timestamp": datetime.now(UTC).isoformat(),
                },
            )


async def continue_generation_session(
    session_id: uuid.UUID,
) -> BenchmarkGeneratorState | None:
    """
    Resumes a paused generation session from the 'coder' node.
    """
    logger.info("continuing_generation_session", session_id=session_id)

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, session_id)
        if not episode:
            logger.error("episode_not_found_for_resume", session_id=session_id)
            return None

        from shared.models.schemas import EpisodeMetadata

        prompt = episode.task
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        custom_objectives = (
            metadata.custom_objectives.model_dump()
            if metadata.custom_objectives
            else {}
        )
        saved_turn_count = int((metadata.additional_info or {}).get("turn_count") or 0)

        # Reconstruct session with status 'EXECUTING' so define_graph routes to 'coder'
        session = GenerationSession(
            session_id=session_id,
            prompt=prompt,
            status=SessionStatus.EXECUTING,
            custom_objectives=custom_objectives,
            validation_logs=metadata.validation_logs,
        )

        # Update episode status
        episode.status = EpisodeStatus.RUNNING
        metadata.detailed_status = SessionStatus.EXECUTING
        episode.metadata_vars = metadata.model_dump()
        await db.commit()

    initial_state = BenchmarkGeneratorState(
        session=session,
        episode_id=str(session_id),
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        turn_count=saved_turn_count,
        plan=None,
        messages=[],
    )

    app = define_graph()
    await mark_episode_execution_window_start(session_id)

    try:
        final_state = await _execute_graph_streaming(
            app, initial_state, session_id, prompt
        )

        await _persist_session_assets(final_state, session_id)

        return final_state
    except Exception as e:
        logger.warning("resume_failed", error=str(e))

    return None
