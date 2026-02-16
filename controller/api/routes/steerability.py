from fastapi import APIRouter, Path, status

from controller.services.steerability.service import steerability_service
from shared.models.steerability import SteerablePrompt

router = APIRouter(prefix="/sessions/{session_id}", tags=["steerability"])


@router.post("/steer", status_code=status.HTTP_202_ACCEPTED)
async def steer_agent(
    session_id: str = Path(..., description="The agent session ID"),
    prompt: SteerablePrompt = ...,
):
    """
    Enqueue a steered prompt for the agent.
    If the agent is idle, it will be picked up immediately (implemented in WP04).
    """
    # TODO: Check if agent is idle and process immediately (WP04)
    queue_position = await steerability_service.enqueue_prompt(session_id, prompt)
    return {"status": "queued", "queue_position": queue_position}


@router.get("/queue", response_model=list[SteerablePrompt])
async def get_steering_queue(
    session_id: str = Path(..., description="The agent session ID"),
):
    """
    Return all currently queued prompts for this session.
    """
    return await steerability_service.get_queued_prompts(session_id)
