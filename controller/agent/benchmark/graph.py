from typing import Literal
import uuid
from uuid import uuid4

import structlog
from langgraph.graph import END, START, StateGraph
from sqlalchemy import update

from controller.persistence.db import get_sessionmaker

from controller.persistence.models import Episode
from shared.enums import EpisodeStatus
from .models import GenerationSession, SessionStatus
from .nodes import coder_node, planner_node, reviewer_node, validator_node
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
    workflow.add_node("validator", validator_node)
    workflow.add_node("reviewer", reviewer_node)

    # Define transitions
    workflow.add_edge(START, "planner")
    workflow.add_edge("planner", "coder")
    workflow.add_edge("coder", "validator")

    # Conditional edges
    def after_validator(state: BenchmarkGeneratorState) -> Literal["coder", "reviewer"]:
        if state.get("simulation_result") and state["simulation_result"]["valid"]:
            return "reviewer"
        return "coder"

    workflow.add_conditional_edges(
        "validator", after_validator, {"coder": "coder", "reviewer": "reviewer"}
    )

    def after_reviewer(state: BenchmarkGeneratorState) -> Literal["coder", "END"]:
        feedback = state.get("review_feedback", "")
        if feedback == "Approved":
            return "END"
        # Optional: Add retry limit check here
        return "coder"

    workflow.add_conditional_edges(
        "reviewer", after_reviewer, {"coder": "coder", "END": END}
    )

    return workflow.compile()


async def run_generation_session(
    prompt: str, session_id: uuid.UUID | None = None
) -> BenchmarkGeneratorState:
    """
    Entry point to run the full generation pipeline with persistence.
    """
    session_id = session_id or uuid4()
    logger.info("running_generation_session", session_id=session_id, prompt=prompt)

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
            },
        )
        db.add(episode)
        await db.commit()

    # 2. Setup State
    session = GenerationSession(
        session_id=session_id, prompt=prompt, status=SessionStatus.planning
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
    final_state = initial_state

    # Helper to map internal status to EpisodeStatus
    def map_status(s: SessionStatus) -> EpisodeStatus:
        if s == SessionStatus.accepted:
            return EpisodeStatus.COMPLETED
        if s == SessionStatus.failed:
            return EpisodeStatus.FAILED
        # All other states (planning, executing, validating, rejected) are RUNNING
        return EpisodeStatus.RUNNING

    try:
        # 3. Stream execution and checkpoint
        async for output in app.astream(initial_state):
            for node_name, state in output.items():
                final_state.update(state)

                # Determine new status
                new_status = SessionStatus.executing  # Default active status

                if node_name == "planner":
                    new_status = SessionStatus.executing
                elif node_name == "coder":
                    new_status = SessionStatus.validating
                elif node_name == "validator":
                    # If valid, it moves to reviewer (still validating/reviewing)
                    # If invalid, moves back to coder (executing)
                    if (
                        state.get("simulation_result")
                        and state["simulation_result"]["valid"]
                    ):
                        new_status = SessionStatus.validating
                    else:
                        new_status = SessionStatus.executing
                elif node_name == "reviewer":
                    feedback = state.get("review_feedback", "")
                    if feedback == "Approved":
                        new_status = SessionStatus.accepted
                    else:
                        new_status = (
                            SessionStatus.rejected
                        )  # Temporarily rejected, will retry

                # Update internal session status
                final_state["session"].status = new_status

                # Update DB (Episode)
                async with session_factory() as db:
                    episode_status = map_status(new_status)

                    # We use Episode instead of GenerationSessionModel so it's visible in UI
                    # Since we are the only writer to this episode, overwriting is fine.
                    # Here we can just overwrite based on local state.

                    current_logs = final_state["session"].validation_logs or []

                    stmt = (
                        update(Episode)
                        .where(Episode.id == session_id)
                        .values(
                            status=episode_status,
                            metadata_vars={
                                "detailed_status": new_status,
                                "validation_logs": current_logs,
                                "prompt": prompt,
                            },
                        )
                    )
                    await db.execute(stmt)
                    await db.commit()

    except Exception as e:
        logger.error("generation_session_failed", session_id=session_id, error=str(e))
        error_msg = f"Error: {e!s}"

        # Update local state
        if "session" in final_state:
            final_state["session"].status = SessionStatus.failed
            if final_state["session"].validation_logs is None:
                final_state["session"].validation_logs = []
            final_state["session"].validation_logs.append(error_msg)

        async with session_factory() as db:
            # Retrieve current logs from state
            current_logs = final_state["session"].validation_logs

            stmt = (
                update(Episode)
                .where(Episode.id == session_id)
                .values(
                    status=EpisodeStatus.FAILED,
                    metadata_vars={
                        "detailed_status": SessionStatus.failed,
                        "validation_logs": current_logs,
                        "prompt": prompt,
                    },
                )
            )
            await db.execute(stmt)
            await db.commit()
        return final_state

    # 4. Final Asset Persistence
    if final_state["session"].status == SessionStatus.accepted:
        try:
            async with session_factory() as db:
                storage = BenchmarkStorage()

                sim_result = final_state.get("simulation_result", {})
                images = sim_result.get("render_data", [])

                # If render_data is None or empty but paths exist (e.g. legacy/error),
                # we depend on validator_node to have populated it.
                # If images is None, initialize to empty list
                if images is None:
                    images = []

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
        except Exception as e:
            logger.error(
                "asset_persistence_failed", session_id=session_id, error=str(e)
            )

    logger.info(
        "generation_session_complete",
        session_id=session_id,
        status=final_state["session"].status,
    )
    return final_state
