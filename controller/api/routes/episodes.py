import asyncio
import uuid
from datetime import datetime
from typing import Annotated

import httpx
import structlog
from fastapi import (
    APIRouter,
    Depends,
    HTTPException,
    Path,
    Query,
    Response,
    WebSocket,
    WebSocketDisconnect,
)
from pydantic import BaseModel, ConfigDict, Field, StrictStr, field_validator
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from controller.api.manager import manager, task_tracker
from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from controller.observability.langfuse import get_langfuse_client
from controller.persistence.db import get_db
from controller.persistence.models import Episode, Trace
from shared.enums import AssetType, EpisodeStatus, ResponseStatus, TraceType

logger = structlog.get_logger(__name__)


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


@router.get("/{episode_id}/assets/{path:path}")
async def get_episode_asset(
    episode_id: uuid.UUID,
    path: str,
    db: AsyncSession = Depends(get_db),
):
    """Proxy asset requests to the worker."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    worker_session_id = None
    if episode.metadata_vars:
        worker_session_id = episode.metadata_vars.get("worker_session_id")

    if not worker_session_id:
        # Fallback for older episodes or benchmarks where session_id might be the id itself
        worker_session_id = str(episode_id)

    worker_url = settings.worker_url
    asset_url = f"{worker_url}/assets/{path}"

    async with httpx.AsyncClient() as client:
        try:
            resp = await client.get(
                asset_url, headers={"X-Session-ID": worker_session_id}, timeout=10.0
            )
            if resp.status_code == 404:
                raise HTTPException(
                    status_code=404, detail=f"Asset {path} not found on worker"
                )
            resp.raise_for_status()

            return Response(
                content=resp.content,
                media_type=resp.headers.get("content-type"),
                status_code=resp.status_code,
            )
        except httpx.HTTPStatusError as e:
            raise HTTPException(
                status_code=e.response.status_code, detail=str(e)
            ) from e
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Failed to proxy asset: {e!s}"
            ) from e


@router.post("/{episode_id}/traces/{trace_id}/feedback", status_code=202)
async def report_trace_feedback(
    episode_id: uuid.UUID,
    trace_id: Annotated[int, Path(le=2147483647)],  # Postgres Integer max
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

    # Langfuse update (best effort)
    if trace.langfuse_trace_id:
        try:
            langfuse = get_langfuse_client()
            if langfuse:
                langfuse.create_score(
                    trace_id=trace.langfuse_trace_id,
                    name="user-feedback",
                    value=feedback.score,
                    comment=feedback.comment,
                )
        except Exception as e:
            logger.warning("langfuse_feedback_failed", error=str(e))

    # Store locally
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


class MessageRequest(BaseModel):
    message: StrictStr
    metadata_vars: dict | None = Field(
        None, description="Additional metadata for the message."
    )

    @field_validator("message")
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
    from shared.observability.events import emit_event
    from shared.observability.schemas import ReviewEvent
    from worker.utils.file_validation import validate_review_frontmatter

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


@router.post("/{episode_id}/messages", status_code=202)
async def continue_episode(
    episode_id: uuid.UUID,
    request: MessageRequest,
    db: AsyncSession = Depends(get_db),
):
    """Send a follow-up message to a running or completed episode."""
    from controller.api.tasks import continue_agent_task

    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    # In a real system, we might want to check if the agent is already busy
    # but for now we'll rely on the task_tracker or just allow it to queue.

    task = asyncio.create_task(
        continue_agent_task(
            episode_id,
            request.message,
            metadata=request.metadata_vars,
        )
    )
    task_tracker.register_task(episode_id, task)

    return {"status": ResponseStatus.ACCEPTED, "message": "Message sent to agent"}


@router.get("/{episode_id}/electronics/schematic")
async def get_episode_schematic(
    episode_id: uuid.UUID,
    db: AsyncSession = Depends(get_db),
):
    """Get the electronics schematic for an episode in a format compatible with tscircuit."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    worker_session_id = str(episode_id)
    if episode.metadata_vars:
        worker_session_id = (
            episode.metadata_vars.get("worker_session_id") or worker_session_id
        )

    worker_url = settings.worker_url
    client = WorkerClient(base_url=worker_url, session_id=worker_session_id)

    try:
        content = await client.read_file("assembly_definition.yaml")
        import yaml

        from shared.models.schemas import AssemblyDefinition

        data = yaml.safe_load(content)
        assembly = AssemblyDefinition(**data)

        if not assembly.electronics:
            return []

        # Map to tscircuit Soup JSON format
        soup = []

        # 1. Add components
        for i, comp in enumerate(assembly.electronics.components):
            # schematic_component
            comp_id = f"comp_{comp.component_id}"

            # Simple layout heuristic (Issue 7)
            y_pos = 10
            if comp.type == "power_supply":
                y_pos = -30
            elif comp.type in ["motor", "relay"]:
                y_pos = 50

            soup.append(
                {
                    "type": "schematic_component",
                    "id": comp_id,
                    "name": comp.component_id,
                    "center": {"x": 10 + i * 50, "y": y_pos},
                    "rotation": 0,
                    "symbol_name": "resistor"
                    if comp.type == "motor"
                    else "generic_component",
                }
            )

            # Add some pins
            soup.append(
                {
                    "type": "schematic_pin",
                    "id": f"{comp_id}_p1",
                    "component_id": comp_id,
                    "name": "1",
                    "center": {"x": 10 + i * 50 - 10, "y": y_pos},
                }
            )
            soup.append(
                {
                    "type": "schematic_pin",
                    "id": f"{comp_id}_p2",
                    "component_id": comp_id,
                    "name": "2",
                    "center": {"x": 10 + i * 50 + 10, "y": y_pos},
                }
            )

        # 2. Add traces (simplified)
        for wire in assembly.electronics.wiring:
            soup.append(
                {
                    "type": "schematic_trace",
                    "id": f"trace_{wire.wire_id}",
                    "source": f"comp_{wire.from_terminal.component}_p1",  # Simplified mapping
                    "target": f"comp_{wire.to_terminal.component}_p2",
                }
            )

        return soup
    except Exception as e:
        logger.error("failed_to_get_schematic", episode_id=episode_id, error=str(e))
        return []


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
    validation_logs: list[str] | None = None
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
        return result.scalars().all()
    except Exception as e:
        logger.error("list_episodes_failed", error=str(e))
        raise HTTPException(
            status_code=500, detail=f"Internal Server Error: {e!s}"
        ) from e


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


@router.delete("/{episode_id}")
async def delete_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Delete an episode and its associated data."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()
    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    try:
        await db.delete(episode)
        await db.commit()
    except Exception as e:
        logger.error("delete_episode_failed", episode_id=str(episode_id), error=str(e))
        await db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to delete episode: {e!s}")

    return {
        "status": ResponseStatus.SUCCESS,
        "message": f"Episode {episode_id} and its traces/assets deleted.",
    }


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
        raise e from None


@router.post("/{episode_id}/interrupt")
async def interrupt_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Interrupt a running episode."""
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

    if episode and episode.status in [EpisodeStatus.RUNNING]:
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
