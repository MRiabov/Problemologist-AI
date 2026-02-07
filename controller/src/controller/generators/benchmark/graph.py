import structlog
from typing import Literal
from uuid import uuid4
from pathlib import Path
from sqlalchemy import update

from langgraph.graph import StateGraph, END, START

from .state import BenchmarkGeneratorState
from .nodes import planner_node, coder_node, validator_node, reviewer_node
from .models import GenerationSession, SessionStatus
from .schema import GenerationSessionModel
from .storage import BenchmarkStorage
from controller.persistence.db import get_db

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
        "validator",
        after_validator,
        {
            "coder": "coder",
            "reviewer": "reviewer"
        }
    )

    def after_reviewer(state: BenchmarkGeneratorState) -> Literal["coder", "END"]:
        feedback = state.get("review_feedback", "")
        if feedback == "Approved":
            return "END"
        # Optional: Add retry limit check here
        return "coder"

    workflow.add_conditional_edges(
        "reviewer",
        after_reviewer,
        {
            "coder": "coder",
            "END": END
        }
    )

    return workflow.compile()

async def run_generation_session(prompt: str) -> BenchmarkGeneratorState:
    """
    Entry point to run the full generation pipeline with persistence.
    """
    session_id = uuid4()
    logger.info("running_generation_session", session_id=session_id, prompt=prompt)
    
    # 1. Create DB entry
    async with get_db() as db:
        db_session = GenerationSessionModel(
            session_id=session_id,
            prompt=prompt,
            status=SessionStatus.planning
        )
        db.add(db_session)
        await db.commit()
    
    # 2. Setup State
    session = GenerationSession(
        session_id=session_id,
        prompt=prompt,
        status=SessionStatus.planning
    )
    
    initial_state = BenchmarkGeneratorState(
        session=session,
        current_script="",
        simulation_result=None,
        review_feedback=None,
        plan=None,
        messages=[]
    )
    
    app = define_graph()
    final_state = initial_state
    
    # 3. Stream execution and checkpoint
    async for output in app.astream(initial_state):
        for node_name, state in output.items():
            final_state = state
            
            # Determine new status
            new_status = SessionStatus.executing # Default active status
            
            if node_name == "planner":
                new_status = SessionStatus.executing
            elif node_name == "coder":
                new_status = SessionStatus.validating
            elif node_name == "validator":
                 # If valid, it moves to reviewer (still validating/reviewing)
                 # If invalid, moves back to coder (executing)
                 if state.get("simulation_result") and state["simulation_result"]["valid"]:
                     new_status = SessionStatus.validating
                 else:
                     new_status = SessionStatus.executing
            elif node_name == "reviewer":
                feedback = state.get("review_feedback", "")
                if feedback == "Approved":
                    new_status = SessionStatus.accepted
                else:
                    new_status = SessionStatus.rejected # Temporarily rejected, will retry

            # Update DB
            async with get_db() as db:
                stmt = (
                    update(GenerationSessionModel)
                    .where(GenerationSessionModel.session_id == session_id)
                    .values(status=new_status)
                )
                await db.execute(stmt)
                await db.commit()

            # Update local session object
            final_state["session"].status = new_status
    
    # 4. Final Asset Persistence
    if final_state["session"].status == SessionStatus.accepted:
        try:
            async with get_db() as db:
                storage = BenchmarkStorage()
                
                sim_result = final_state.get("simulation_result", {})
                images = []
                if sim_result:
                    for path_str in sim_result.get("render_paths", []):
                        try:
                            images.append(Path(path_str).read_bytes())
                        except Exception as e:
                            logger.warn("image_read_failed", path=path_str, error=str(e))
                
                mjcf_content = final_state.get("mjcf_content", "<!-- MJCF content missing in state -->")
                
                await storage.save_asset(
                    benchmark_id=session_id,
                    script=final_state["current_script"],
                    mjcf=mjcf_content,
                    images=images,
                    metadata=final_state.get("plan", {}),
                    db=db
                )
                logger.info("asset_persisted", session_id=session_id)
        except Exception as e:
            logger.error("asset_persistence_failed", session_id=session_id, error=str(e))

    logger.info("generation_session_complete", session_id=session_id, status=final_state["session"].status)
    return final_state
