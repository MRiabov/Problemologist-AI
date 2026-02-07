from datetime import datetime
from typing import List, Optional
from sqlalchemy import String, DateTime, JSON, ForeignKey, Integer, Enum as SQLEnum
from sqlalchemy.orm import Mapped, mapped_column, relationship
from .db import Base
import uuid

from src.shared.enums import EpisodeStatus, AssetType


class Episode(Base):
    __tablename__ = "episodes"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    task: Mapped[str] = mapped_column(String)
    status: Mapped[EpisodeStatus] = mapped_column(
        SQLEnum(EpisodeStatus),
        default=EpisodeStatus.RUNNING,
        insert_default=EpisodeStatus.RUNNING,
    )
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, default=datetime.utcnow, onupdate=datetime.utcnow
    )

    # Reproducibility metadata
    skill_git_hash: Mapped[Optional[str]] = mapped_column(String)
    template_versions: Mapped[Optional[dict]] = mapped_column(JSON)
    metadata_vars: Mapped[Optional[dict]] = mapped_column(JSON)
    
    # Agent state
    todo_list: Mapped[Optional[dict]] = mapped_column(JSON)
    journal: Mapped[Optional[str]] = mapped_column(String)
    plan: Mapped[Optional[str]] = mapped_column(String)

    traces: Mapped[List["Trace"]] = relationship(
        back_populates="episode", cascade="all, delete-orphan"
    )
    assets: Mapped[List["Asset"]] = relationship(
        back_populates="episode", cascade="all, delete-orphan"
    )


class Trace(Base):
    __tablename__ = "traces"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    episode_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("episodes.id"))
    langfuse_trace_id: Mapped[Optional[str]] = mapped_column(String)
    raw_trace: Mapped[Optional[dict]] = mapped_column(JSON)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

    episode: Mapped["Episode"] = relationship(back_populates="traces")


class Asset(Base):
    __tablename__ = "assets"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    episode_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("episodes.id"))
    asset_type: Mapped[AssetType] = mapped_column(SQLEnum(AssetType))
    s3_path: Mapped[str] = mapped_column(String)
    content: Mapped[Optional[str]] = mapped_column(String, nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

    episode: Mapped["Episode"] = relationship(back_populates="assets")
