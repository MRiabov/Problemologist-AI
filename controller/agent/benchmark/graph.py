import os
import uuid
from typing import Any, Literal
from uuid import uuid4

import httpx
import structlog
from langgraph.graph import END, START, StateGraph
from sqlalchemy import select

from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import get_langfuse_callback
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode
from shared.enums import AssetType, EpisodeStatus
from shared.simulation.schemas import SimulatorBackendType
from controller.graph.steerability_node import steerability_node, check_steering

from .models import GenerationSession, SessionStatus
from .nodes import (
    coder_node,
    cots_search_node,
    planner_node,
    reviewer_node,
    skills_node,
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
    workflow.add_node("steer", steerability_node)

    # Define transitions
    def route_start(state: BenchmarkGeneratorState) -> Literal["planner", "coder"]:
        if state["session"].status == SessionStatus.executing:
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
    ) -> Literal["steer", "coder", "planner", "skills"]:
        # Check for steering first
        if await check_steering(state) == "steer":
            return "steer"

        feedback = state.get("review_feedback", "")
        if feedback == "Approved":
            return "skills"
        if feedback.startswith("Steering:"):
            return "planner"
        return "coder"

    workflow.add_conditional_edges(
        "reviewer",
        reviewer_router,
        {"steer": "steer", "coder": "coder", "planner": "planner", "skills": "skills"},
    )

    workflow.add_edge("skills", END)
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
    session_factory = get_sessionmaker()
    final_state = initial_state

    # Helper to map internal status to EpisodeStatus
    def map_status(s: SessionStatus) -> EpisodeStatus:
        if s == SessionStatus.accepted:
            return EpisodeStatus.COMPLETED
        if s == SessionStatus.failed:
            return EpisodeStatus.FAILED
        if s == SessionStatus.planned:
            return EpisodeStatus.PLANNED
        return EpisodeStatus.RUNNING

    # Langfuse tracing
    langfuse_callback = get_langfuse_callback(
        name="benchmark_generator", session_id=str(session_id)
    )
    db_callback = DatabaseCallbackHandler(episode_id=str(session_id))
    callbacks = [db_callback]
    if langfuse_callback:
        callbacks.append(langfuse_callback)

    async for output in app.astream(initial_state, config={"callbacks": callbacks}):
        for node_name, state in output.items():
            final_state.update(state)

            # Determine new status
            new_status = final_state["session"].status

            should_stop = False
            if node_name == "planner":
                if state.get("plan") and "error" not in state["plan"]:
                    new_status = SessionStatus.planned
                    should_stop = True
                else:
                    new_status = SessionStatus.failed
            elif node_name == "coder":
                new_status = SessionStatus.validating
            elif node_name == "reviewer":
                feedback = state.get("review_feedback", "")
                if feedback == "Approved":
                    new_status = SessionStatus.accepted
                else:
                    new_status = SessionStatus.rejected
            elif (
                node_name == "skills"
                and final_state["session"].status == SessionStatus.accepted
            ):
                # If we've approved everything, skills is the final step
                new_status = SessionStatus.accepted

            # Update internal session status
            final_state["session"].status = new_status

            # Update DB (Episode)
            async with session_factory() as db:
                episode_status = map_status(new_status)

                stmt_select = select(Episode).where(Episode.id == session_id)
                res = await db.execute(stmt_select)
                ep = res.scalar_one_or_none()

                if ep:
                    new_metadata = (ep.metadata_vars or {}).copy()
                    new_metadata.update(
                        {
                            "detailed_status": new_status,
                            "validation_logs": final_state["session"].validation_logs
                            or [],
                            "prompt": prompt,
                        }
                    )

                    ep.status = episode_status
                    ep.metadata_vars = new_metadata

                    # Update plan if it exists in state
                    if final_state.get("plan"):
                        import json

                        ep.plan = json.dumps(final_state["plan"], indent=2)

                    await db.commit()

            if should_stop:
                logger.info("pausing_for_user_confirmation", session_id=session_id)
                return final_state

    return final_state


async def run_generation_session(
    prompt: str,
    session_id: uuid.UUID | None = None,
    custom_objectives: dict | None = None,
    backend: SimulatorBackendType = SimulatorBackendType.MUJOCO,
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
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = Episode(
            id=session_id,
            task=prompt,
            status=EpisodeStatus.RUNNING,
            metadata_vars={
                "detailed_status": SessionStatus.planning,
                "validation_logs": [],
                "prompt": prompt,
                "custom_objectives": custom_objectives,
                "backend": backend,
            },
        )
        db.add(episode)
        await db.commit()

    # 2. Setup State
    session = GenerationSession(
        session_id=session_id,
        prompt=prompt,
        status=SessionStatus.planning,
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
        initial_state["session"].status = SessionStatus.failed
        # Update DB (Episode) to failed
        async with session_factory() as db:
            episode = await db.get(Episode, session_id)
            if episode:
                episode.status = EpisodeStatus.FAILED
                metadata = (episode.metadata_vars or {}).copy()
                metadata.update(
                    {
                        "detailed_status": SessionStatus.failed,
                        "error": str(e),
                    }
                )
                episode.metadata_vars = metadata
                await db.commit()
        return initial_state

    # 4. Final Asset Persistence
    await _persist_session_assets(final_state, session_id)

    logger.info(
        "generation_session_complete",
        session_id=session_id,
        status=final_state["session"].status,
    )
    return final_state


async def _persist_session_assets(
    final_state: BenchmarkGeneratorState, session_id: uuid.UUID
):
    """Helper to persist final assets after a successful session."""
    if final_state["session"].status != SessionStatus.accepted:
        return

    session_factory = get_sessionmaker()
    try:
        async with session_factory() as db:
            storage = BenchmarkStorage()

            sim_result = final_state.get("simulation_result", {})
            images = sim_result.get("render_data", []) or []

            mjcf_content = final_state.get(
                "mjcf_content", "<!-- MJCF content missing in state -->"
            )

            await storage.save_asset(
                benchmark_id=session_id,
                script=final_state["current_script"],
                mjcf=mjcf_content,
                images=images,
                metadata=final_state.get("plan", {}),
                db=db,
            )
            logger.info("asset_persisted", session_id=session_id)

            # Sync assets to the Asset table
            try:
                from contextlib import suppress

                worker_url = os.getenv("WORKER_URL", "http://worker:8001")
                async with httpx.AsyncClient() as http_client:
                    client = WorkerClient(
                        base_url=worker_url,
                        session_id=str(session_id),
                        http_client=http_client,
                    )
                    middleware = RemoteFilesystemMiddleware(client)
                    backend = RemoteFilesystemBackend(middleware)

                    files = await backend.als_info("/")
                    for file_info in files:
                        if not file_info["is_dir"]:
                            path = file_info["path"]
                            asset_type = AssetType.OTHER
                            if path.endswith(".py"):
                                asset_type = AssetType.PYTHON
                            elif path.endswith(".xml") or path.endswith(".mjcf"):
                                asset_type = AssetType.MJCF

                            content = None
                            with suppress(Exception):
                                raw_content = await backend.aread(path)
                                if isinstance(raw_content, bytes):
                                    content = raw_content.decode(
                                        "utf-8", errors="replace"
                                    )
                                else:
                                    content = str(raw_content)

                            asset = Asset(
                                episode_id=session_id,
                                asset_type=asset_type,
                                s3_path=path,
                                content=content,
                            )
                            db.add(asset)
                    await db.commit()
            except Exception as e:
                logger.error("failed_to_sync_assets_to_db", error=str(e))
    except Exception as e:
        logger.error("asset_persistence_failed", error=str(e))


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

        prompt = episode.task
        metadata = episode.metadata_vars or {}
        custom_objectives = metadata.get("custom_objectives", {})

        # Reconstruct session with status 'executing' so define_graph routes to 'coder'
        session = GenerationSession(
            session_id=session_id,
            prompt=prompt,
            status=SessionStatus.executing,
            custom_objectives=custom_objectives,
            validation_logs=metadata.get("validation_logs", []),
        )

        # Update episode status
        episode.status = EpisodeStatus.RUNNING
        metadata["detailed_status"] = SessionStatus.executing
        episode.metadata_vars = metadata
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
