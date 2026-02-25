import os
import uuid
from datetime import datetime, UTC
from typing import Any, Literal
from uuid import uuid4

import httpx
import structlog
from langgraph.graph import END, START, StateGraph

from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.graph.steerability_node import check_steering, steerability_node
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.persistence.db import get_sessionmaker
from opentelemetry import trace
from controller.persistence.models import Asset, Episode
from shared.enums import AssetType, EpisodeStatus, ReviewDecision
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


def define_graph():
    """
    Constructs the LangGraph for benchmark scenario generation.
    """
    workflow = StateGraph(BenchmarkGeneratorState)

    # Add nodes
    workflow.add_node("planner", planner_node)
    workflow.add_node("coder", coder_node)
    workflow.add_node("reviewer", reviewer_node)
    workflow.add_node("cots_search", cots_search_node)
    workflow.add_node("skills", skills_node)
    workflow.add_node("summarizer", summarizer_node)
    workflow.add_node("steer", steerability_node)

    # Define transitions
    def route_start(state: BenchmarkGeneratorState) -> Literal["planner", "coder"]:
        if state.session.status == SessionStatus.EXECUTING:
            return "coder"
        return "planner"

    workflow.add_conditional_edges(START, route_start)
    workflow.add_edge("planner", "coder")

    # In benchmark coder also does validation (it was coder -> validator -> reviewer)
    workflow.add_conditional_edges(
        "coder",
        check_steering,
        {"steer": "steer", "next": "reviewer"},
    )

    # Conditional edges for reviewer
    async def reviewer_router(
        state: BenchmarkGeneratorState,
    ) -> Literal["steer", "coder", "planner", "skills", "summarizer"]:
        # Check for steering first
        if await check_steering(state) == "steer":
            return "steer"

        # Check for summarization need
        if len(state.journal or "") > 5000:
            return "summarizer"

        if state.review_round > 10:
            logger.warning(
                "max_review_rounds_reached", session_id=state.session.session_id
            )
            return "skills"

        # Use structured decision if available
        if state.review_decision:
            if state.review_decision == ReviewDecision.APPROVED:
                return "skills"
            if state.review_decision == ReviewDecision.REJECT_PLAN:
                return "planner"
            return "coder"

        # Fallback for legacy behavior
        feedback = (state.review_feedback or "").upper()
        if "APPROVED" in feedback:
            return "skills"
        if feedback.startswith("STEERING:"):
            return "planner"
        return "coder"

    workflow.add_conditional_edges(
        "reviewer",
        reviewer_router,
        {
            "steer": "steer",
            "coder": "coder",
            "planner": "planner",
            "skills": "skills",
            "summarizer": "summarizer",
        },
    )

    workflow.add_edge("skills", END)
    workflow.add_edge("summarizer", "planner")
    workflow.add_edge("steer", "planner")

    # cots_search can be reached from planner or coder if we add those edges
    workflow.add_edge("cots_search", "planner")

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
    db_callback = DatabaseCallbackHandler(episode_id=str(session_id))
    callbacks = [db_callback]

    async for output in app.astream(initial_state, config={"callbacks": callbacks}):
        for node_name, state_update in output.items():
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

            if node_name == "planner":
                if final_state.plan and final_state.plan.theme != "error":
                    new_status = SessionStatus.PLANNED
                    should_stop = True
                else:
                    new_status = SessionStatus.FAILED
            elif node_name == "coder":
                new_status = SessionStatus.VALIDATING
            elif node_name == "reviewer":
                if final_state.review_decision == ReviewDecision.APPROVED:
                    new_status = SessionStatus.ACCEPTED
                elif final_state.review_decision:
                    new_status = SessionStatus.REJECTED
                else:
                    feedback = final_state.review_feedback or ""
                    if "APPROVED" in feedback.upper():
                        new_status = SessionStatus.ACCEPTED
                    else:
                        new_status = SessionStatus.REJECTED
            elif (
                node_name == "skills"
                and final_state.session.status == SessionStatus.ACCEPTED
            ):
                new_status = SessionStatus.ACCEPTED

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
                )
            except Exception as e:
                logger.error("failed_to_update_episode_persistence", error=str(e))

            if should_stop:
                logger.info("pausing_for_user_confirmation", session_id=session_id)
                return final_state

    # Report automated score to Langfuse
    try:
        from controller.observability.langfuse import (
            calculate_and_report_automated_score,
        )

        # Get trace_id from OpenTelemetry context
        trace_id = None
        span = trace.get_current_span()
        if span and span.get_span_context().is_valid:
            trace_id = f"{span.get_span_context().trace_id:032x}"

        if trace_id:
            async with get_sessionmaker()() as db:
                from controller.config.settings import settings as global_settings

                worker_light_url = global_settings.worker_light_url
                client = WorkerClient(
                    base_url=worker_light_url,
                    session_id=str(session_id),
                    heavy_url=global_settings.worker_heavy_url,
                )
                await calculate_and_report_automated_score(
                    episode_id=session_id,
                    trace_id=trace_id,
                    agent_name="benchmark_generator",
                    db=db,
                    worker_client=client,
                )
    except Exception as e:
        logger.error("failed_to_report_benchmark_automated_score", error=str(e))

    return final_state


async def run_generation_session(
    prompt: str,
    session_id: uuid.UUID | None = None,
    custom_objectives: CustomObjectives | None = None,
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
    from shared.models.schemas import EpisodeMetadata

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        metadata = EpisodeMetadata(
            detailed_status=SessionStatus.PLANNING,
            validation_logs=[],
            prompt=prompt,
            custom_objectives=custom_objectives,
            episode_type="benchmark",
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
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        plan=None,
        messages=[],
    )

    app = define_graph()

    try:
        # 3. Stream execution and checkpoint
        final_state = await _execute_graph_streaming(
            app, initial_state, session_id, prompt
        )
    except Exception as e:
        logger.error("generation_session_failed", session_id=session_id, error=str(e))
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
                episode.metadata_vars = metadata.model_dump()
                await db.commit()
        return initial_state

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
    """Helper to persist final assets after a successful session."""
    if final_state.session.status != SessionStatus.ACCEPTED:
        return

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

            # Sync assets to the Asset table
            try:
                from contextlib import suppress
                from controller.config.settings import settings as global_settings

                worker_light_url = global_settings.worker_light_url
                async with httpx.AsyncClient() as http_client:
                    client = WorkerClient(
                        base_url=worker_light_url,
                        session_id=str(session_id),
                        http_client=http_client,
                        heavy_url=global_settings.worker_heavy_url,
                    )
                    middleware = RemoteFilesystemMiddleware(client)
                    backend = RemoteFilesystemBackend(middleware)

                    from controller.observability.middleware_helper import (
                        broadcast_file_update,
                    )

                    async def sync_dir(dir_path: str):
                        # Avoid infinite recursion and stay in workspace
                        if dir_path.startswith(
                            ("/utils", "/reviews", "/config", "/skills", "/.git")
                        ):
                            return

                        files = await backend.als_info(dir_path)
                        logger.info(
                            "syncing_benchmark_assets",
                            session_id=session_id,
                            path=dir_path,
                            count=len(files),
                        )

                        for file_info in files:
                            path = file_info["path"]
                            if file_info["is_dir"]:
                                # Recurse into subdirectories like assets/ and renders/
                                if not path.endswith("/"):
                                    path += "/"
                                await sync_dir(path)
                            else:
                                # Skip obviously irrelevant files
                                if path.endswith((".log", ".lock", ".tmp", ".pyc")):
                                    continue

                                # Determine if we should read content
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
                                    with suppress(Exception):
                                        raw_content = await backend.aread(path)
                                        if isinstance(raw_content, bytes):
                                            content = raw_content.decode(
                                                "utf-8", errors="replace"
                                            )
                                        else:
                                            content = str(raw_content)

                                await broadcast_file_update(
                                    str(session_id), path, content
                                )

                    await sync_dir("/")
                    await db.commit()
            except Exception as e:
                logger.error("failed_to_sync_assets_to_db", error=str(e))
    except Exception as e:
        logger.error("asset_persistence_failed", error=str(e))


async def _update_episode_persistence(
    session_id: uuid.UUID,
    new_status: str,
    validation_logs: list[str],
    prompt: str,
    plan: Any = None,
    journal: str | None = None,
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
            metadata.detailed_status = new_status
            metadata.validation_logs = validation_logs
            metadata.prompt = prompt
            metadata.plan = plan.model_dump() if hasattr(plan, "model_dump") else plan
            episode.metadata_vars = metadata.model_dump()
            if journal:
                episode.journal = journal

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
        current_script="",
        simulation_result=None,
        review_feedback=None,
        review_round=0,
        plan=None,
        messages=[],
    )

    app = define_graph()

    try:
        final_state = await _execute_graph_streaming(
            app, initial_state, session_id, prompt
        )

        await _persist_session_assets(final_state, session_id)

        return final_state
    except Exception as e:
        logger.error("resume_failed", error=str(e))

    return None
