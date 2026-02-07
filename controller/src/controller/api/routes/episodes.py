from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from sqlalchemy.orm import selectinload
from typing import List, Optional
from pydantic import BaseModel, StrictStr
import uuid
import json
import asyncio
from datetime import datetime

from controller.persistence.db import get_db
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, AssetType

router = APIRouter(prefix="/episodes", tags=["episodes"])

class TraceResponse(BaseModel):
    id: int
    langfuse_trace_id: Optional[str]
    raw_trace: Optional[dict]
    created_at: datetime

    class Config:
        from_attributes = True

class AssetResponse(BaseModel):
    id: int
    asset_type: AssetType
    s3_path: str
    created_at: datetime

    class Config:
        from_attributes = True

class EpisodeResponse(BaseModel):
    id: uuid.UUID
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: Optional[str] = None
    template_versions: Optional[dict] = None
    metadata_vars: Optional[dict] = None
    traces: List[TraceResponse] = []
    assets: List[AssetResponse] = []

    class Config:
        from_attributes = True

@router.get("/", response_model=List[EpisodeResponse])
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
    await websocket.accept()
    try:
        # Initial message
        await websocket.send_json({
            "type": "log",
            "data": f"Subscribed to updates for episode {episode_id}",
            "timestamp": datetime.utcnow().isoformat()
        })
        
        # Placeholder for real-time updates. 
        # In a real implementation, we would use a Pub/Sub system (like Redis or Postgres NOTIFY)
        # to push updates from the background workers to this websocket.
        while True:
            # Just keep connection alive for now
            await asyncio.sleep(10)
            await websocket.send_json({
                "type": "log", 
                "data": "Heartbeat...", 
                "timestamp": datetime.utcnow().isoformat()
            })
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"WebSocket error: {e}")