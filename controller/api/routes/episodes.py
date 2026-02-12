import asyncio
import uuid
from datetime import datetime
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, ConfigDict, Field, StrictStr, field_validator
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from controller.api.manager import manager, task_tracker
from controller.observability.langfuse import get_langfuse_client
from controller.persistence.db import get_db
from controller.persistence.models import Episode, Trace
from shared.enums import AssetType, EpisodeStatus, ResponseStatus, TraceType


class FeedbackRequest(BaseModel):
    score: int  # 1 for up, 0 for down
    comment: str | None = None

    @field_validator("comment")
    @classmethod
    def strip_null_bytes(cls, v: str | None) -> str | None:
        if v is not None:
            return v.replace("\u0000", "")
        return v


router = APIRouter(prefix="/episodes", tags=["episodes"])


@router.post("/{episode_id}/traces/{trace_id}/feedback", status_code=202)
async def report_trace_feedback(
    episode_id: uuid.UUID,
    trace_id: Annotated[int, Query(lt=2**63)],
    feedback: FeedbackRequest,
    db: AsyncSession = Depends(get_db),
):
    """Report feedback for a specific trace to Langfuse."""
    result = await db.execute(
        select(Trace).where(Trace.id == trace_id, Trace.episode_id == episode_id)
    )
    trace = result.scalar_one_or_none()

    if not trace:
        raise HTTPException(status_code=404, detail="Trace not found")

    if not trace.langfuse_trace_id:
        raise HTTPException(status_code=400, detail="Trace does not have a Langfuse ID")

    langfuse = get_langfuse_client()
    if not langfuse:
        raise HTTPException(status_code=503, detail="Langfuse client not configured")

    langfuse.create_score(
        trace_id=trace.langfuse_trace_id,
        name="user-feedback",
        value=feedback.score,
        comment=feedback.comment,
    )

    # Store locally anyway (spec requirement)
    trace.feedback_score = feedback.score
    trace.feedback_comment = feedback.comment
    await db.commit()

    return {"status": ResponseStatus.ACCEPTED}


class ReviewRequest(BaseModel):
    review_content: str

    @field_validator("review_content")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


@router.post("/{episode_id}/review")
async def review_episode(
    episode_id: uuid.UUID,
    request: ReviewRequest,
    db: AsyncSession = Depends(get_db),
):
    """Submit a review for an episode."""
    from worker.utils.file_validation import validate_review_frontmatter
    from shared.observability.events import emit_event
    from shared.observability.schemas import ReviewEvent

    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    # Validate the review frontmatter
    is_valid, review_data = validate_review_frontmatter(request.review_content)
    if not is_valid:
        raise HTTPException(status_code=422, detail=f"Invalid review: {review_data}")

    # Update episode based on decision
    decision = review_data.decision
    if decision == "approved":
        episode.status = EpisodeStatus.COMPLETED
    elif decision == "rejected":
        episode.status = EpisodeStatus.FAILED

    # Emit review event
    emit_event(
        ReviewEvent(
            episode_id=str(episode_id),
            decision=decision,
            comments=review_data.comments,
        )
    )

    await db.commit()
    return {"status": ResponseStatus.SUCCESS, "decision": decision}


class TraceResponse(BaseModel):
    id: int
    langfuse_trace_id: str | None
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata: dict | None = Field(None, alias="metadata_vars")
    feedback_score: int | None = None
    feedback_comment: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)


class AssetResponse(BaseModel):
    id: int
    asset_type: AssetType
    s3_path: str
    content: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)


class EpisodeResponse(BaseModel):
    id: uuid.UUID
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    template_versions: dict | None = None
    metadata_vars: dict | None = None
    todo_list: dict | None = None
    journal: str | None = None
    plan: str | None = None
    traces: list[TraceResponse] = []
    assets: list[AssetResponse] = []

    model_config = ConfigDict(from_attributes=True)


@router.get("/", response_model=list[EpisodeResponse])
async def list_episodes(
    limit: Annotated[int, Query(ge=0, lt=2**63)] = 100,
    offset: Annotated[int, Query(ge=0, lt=2**63)] = 0,
    db: AsyncSession = Depends(get_db),
):
    """List all agent episodes."""
    try:
        result = await db.execute(
            select(Episode)
            .order_by(Episode.created_at.desc())
            .limit(limit)
            .offset(offset)
            .options(selectinload(Episode.traces), selectinload(Episode.assets))
        )
        episodes = result.scalars().all()
        return episodes
    except Exception as e:
        import structlog

        logger = structlog.get_logger(__name__)
        logger.error("list_episodes_failed", error=str(e))
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e!s}")


@router.get("/{episode_id}", response_model=EpisodeResponse)
async def get_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Get a specific episode."""
    result = await db.execute(
        select(Episode)
        .where(Episode.id == episode_id)
        .options(selectinload(Episode.traces), selectinload(Episode.assets))
    )
    episode = result.scalar_one_or_none()
    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")
    return episode


@router.websocket("/{episode_id}/ws")
async def episode_websocket(websocket: WebSocket, episode_id: uuid.UUID):
    await manager.connect(episode_id, websocket)
    try:
        # Initial message
        await websocket.send_json(
            {
                "type": "log",
                "data": f"Subscribed to updates for episode {episode_id}",
                "timestamp": datetime.utcnow().isoformat(),
            }
        )

        # Keep connection alive
        while True:
            await asyncio.sleep(30)
            await websocket.send_json({"type": "heartbeat"})
    except WebSocketDisconnect:
        manager.disconnect(episode_id, websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(episode_id, websocket)


@router.post("/{episode_id}/interrupt")
async def interrupt_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Interrupt a running episode."""
    import structlog

    logger = structlog.get_logger(__name__)

    # 1. Cancel the asyncio task
    task = task_tracker.get_task(episode_id)
    if task and not task.done():
        logger.info("cancelling_episode_task", episode_id=str(episode_id))
        task.cancel()
    else:
        logger.warning("task_not_found_or_already_done", episode_id=str(episode_id))

    # 2. Update status in DB immediately (failsafe)
    # The cancelled task exception handler in main.py will also do this,
    # but we do it here to give immediate feedback.
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if episode:
        if episode.status in [EpisodeStatus.RUNNING]:
            episode.status = EpisodeStatus.CANCELLED
            await db.commit()

            # Broadcast update
            await manager.broadcast(
                episode_id,
                {
                    "type": "status_update",
                    "status": EpisodeStatus.CANCELLED,
                    "timestamp": datetime.utcnow().isoformat(),
                },
            )

    return {"status": ResponseStatus.ACCEPTED, "message": "Interruption requested"}