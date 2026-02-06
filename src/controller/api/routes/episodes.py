from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from typing import List
from pydantic import BaseModel
import uuid
from datetime import datetime

from src.controller.persistence.db import get_db
from src.controller.persistence.models import Episode
from src.shared.enums import EpisodeStatus

router = APIRouter(prefix="/episodes", tags=["episodes"])

class EpisodeResponse(BaseModel):
    id: uuid.UUID
    task: str
    status: EpisodeStatus
    created_at: datetime

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
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()
    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")
    return episode
