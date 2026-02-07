import structlog
from typing import Literal
from uuid import uuid4

from langgraph.graph import StateGraph, END, START

from .state import BenchmarkGeneratorState
from .nodes import planner_node, coder_node, validator_node, reviewer_node
from .models import GenerationSession, SessionStatus

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
    Entry point to run the full generation pipeline.
    """
    session_id = uuid4()
    logger.info("running_generation_session", session_id=session_id, prompt=prompt)
    
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
    final_state = await app.ainvoke(initial_state)
    
    # Update status based on final feedback
    if final_state.get("review_feedback") == "Approved":
        final_state["session"].status = SessionStatus.accepted
    else:
        final_state["session"].status = SessionStatus.rejected
        
    logger.info("generation_session_complete", session_id=session_id, status=final_state["session"].status)
    return final_state
