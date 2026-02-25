import asyncio
import uuid

import structlog
from fastapi import APIRouter, Depends, Path, status
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from controller.api.manager import task_tracker
from controller.api.tasks import continue_agent_task
from controller.persistence.db import get_db
from controller.persistence.models import Episode
from controller.services.steerability.service import steerability_service
from shared.models.steerability import SteerablePrompt

logger = structlog.get_logger(__name__)

router = APIRouter(prefix="/sessions/{session_id}", tags=["steerability"])


async def get_episode_from_session_id(
    db: AsyncSession, session_id: str
) -> Episode | None:
    """
    Find the episode associated with a session ID.
    Checks both Episode.id and metadata_vars.worker_session_id.
    """
    # 1. Try parsing as Episode UUID
    try:
        episode_id = uuid.UUID(session_id)
        result = await db.execute(select(Episode).where(Episode.id == episode_id))
        episode = result.scalar_one_or_none()
        if episode:
            return episode
    except ValueError:
        pass

    # 2. Try searching by worker_session_id in metadata
    try:
        # Attempt to use JSON operator for Postgres.
        # This assumes Episode.metadata_vars is a JSON/JSONB column.
        stmt = (
            select(Episode)
            .where(Episode.metadata_vars["worker_session_id"].astext == session_id)
            .order_by(Episode.created_at.desc())
            .limit(1)
        )
        result = await db.execute(stmt)
        episode = result.scalar_one_or_none()
        return episode
    except Exception as e:
        logger.warning("failed_to_query_episode_by_metadata", error=str(e))
        return None


@router.post("/steer", status_code=status.HTTP_202_ACCEPTED)
async def steer_agent(
    session_id: str = Path(..., description="The agent session ID"),
    prompt: SteerablePrompt = ...,
    db: AsyncSession = Depends(get_db),
):
    """
    Enqueue a steered prompt for the agent.
    If the agent is idle, it will be picked up immediately (implemented in WP04).
    """
    episode = await get_episode_from_session_id(db, session_id)

    if episode:
        # Check if running
        task = task_tracker.get_task(episode.id)
        if task and not task.done():
            # Agent is running, enqueue
            logger.info("agent_running_enqueuing_prompt", episode_id=str(episode.id))
            queue_position = await steerability_service.enqueue_prompt(
                session_id, prompt
            )
            return {"status": "queued", "queue_position": queue_position}
        # Agent is idle, wake up
        logger.info("agent_idle_waking_up", episode_id=str(episode.id))

        # Start background task
        new_task = asyncio.create_task(
            continue_agent_task(
                episode.id,
                prompt.text,
                metadata=prompt.model_dump(),
            )
        )
        task_tracker.register_task(episode.id, new_task)

        return {"status": "started", "queue_position": 0}
    logger.warning("episode_not_found_for_steer", session_id=session_id)
    # Fallback to just enqueueing if we can't find the episode object but session exists?
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
