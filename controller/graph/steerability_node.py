from typing import Any

import structlog
from langchain_core.messages import HumanMessage

from controller.agent.state import AgentStatus, AgentState
from controller.services.steerability.service import steerability_service

logger = structlog.get_logger(__name__)


def get_session_id(state: Any) -> str | None:
    """Extract session_id from either AgentState or BenchmarkGeneratorState."""
    if isinstance(state, AgentState):
        return state.session_id
    if type(state).__name__ == "BenchmarkGeneratorState":
        return str(state.session.session_id)

    if isinstance(state, dict):
        session = state.get("session")
        if session:
            if hasattr(session, "session_id"):
                return str(session.session_id)
            if isinstance(session, dict):
                return str(session.get("session_id"))
        return state.get("session_id")

    return None


async def steerability_node(state: Any) -> dict:
    """
    Checks for queued user feedback and injects it into the agent context.
    If feedback is found, it dequeues it and returns a state update that forces re-planning.
    """
    session_id = get_session_id(state)
    if not session_id:
        return {}

    queued_prompts = await steerability_service.get_queued_prompts(session_id)
    if not queued_prompts:
        return {}

    # Dequeue the first prompt
    steerable_prompt = await steerability_service.dequeue_prompt(session_id)

    logger.info(
        "steerability_interruption", session_id=session_id, text=steerable_prompt.text
    )

    # Attach metadata to the message
    metadata = steerable_prompt.model_dump()
    del metadata["text"]

    human_message = HumanMessage(
        content=steerable_prompt.text, additional_kwargs={"steerability": metadata}
    )

    update = {
        "messages": [human_message],
    }

    # Handle status/journal based on state type
    if isinstance(state, dict):
        # BenchmarkGeneratorState
        update["review_feedback"] = f"Steering: {steerable_prompt.text}"
    else:
        # AgentState
        update["status"] = AgentStatus.PLAN_REJECTED
        update["journal"] = (
            getattr(state, "journal", "") + f"\n[User Steering] {steerable_prompt.text}"
        )

    return update


async def check_steering(state: Any) -> str:
    """
    Conditional edge logic to decide if we should interrupt for steering.
    """
    session_id = get_session_id(state)
    if not session_id:
        return "next"

    queued = await steerability_service.get_queued_prompts(session_id)
    if queued:
        return "steer"
    return "next"
