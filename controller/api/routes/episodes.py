import asyncio
import uuid
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, StrictStr
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from controller.api.manager import manager
from controller.persistence.db import get_db
from controller.persistence.models import Episode
from shared.enums import AssetType, EpisodeStatus

router = APIRouter(prefix="/episodes", tags=["episodes"])


class TraceResponse(BaseModel):
    id: int
    langfuse_trace_id: str | None
    raw_trace: dict | None
    created_at: datetime

    class Config:
        from_attributes = True


class AssetResponse(BaseModel):
    id: int
    asset_type: AssetType
    s3_path: str
    content: str | None = None
    created_at: datetime

    class Config:
        from_attributes = True


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

    class Config:
        from_attributes = True


@router.get("/", response_model=list[EpisodeResponse])
async def list_episodes(db: AsyncSession = Depends(get_db)):
    """List all agent episodes."""
    result = await db.execute(select(Episode).order_by(Episode.created_at.desc()))
    episodes = result.scalars().all()
    return episodes


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
