from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from sqlalchemy.orm import selectinload
from typing import List, Optional
from pydantic import BaseModel, StrictStr
import uuid
from datetime import datetime

from src.controller.persistence.db import get_db
from src.controller.persistence.models import Episode
from src.shared.enums import EpisodeStatus, AssetType

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
