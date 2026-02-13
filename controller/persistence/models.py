import uuid
from datetime import datetime

import sqlalchemy as sa
from sqlalchemy import JSON, DateTime, ForeignKey, Integer, String
from sqlalchemy import Enum as SQLEnum
from sqlalchemy.orm import Mapped, mapped_column, relationship

from shared.enums import AssetType, EpisodeStatus, TraceType

from .db import Base


class BenchmarkAsset(Base):
    __tablename__ = "benchmark_assets"

    benchmark_id: Mapped[uuid.UUID] = mapped_column(
        primary_key=True, default=uuid.uuid4, index=True
    )
    mjcf_url: Mapped[str] = mapped_column(String)
    build123d_url: Mapped[str] = mapped_column(String)
    preview_bundle_url: Mapped[str] = mapped_column(String)
    random_variants: Mapped[dict | None] = mapped_column(JSON)
    difficulty_score: Mapped[float | None] = mapped_column(sa.Float)
    benchmark_metadata: Mapped[dict | None] = mapped_column("metadata", JSON)


class GenerationSession(Base):
    __tablename__ = "generation_sessions"

    session_id: Mapped[uuid.UUID] = mapped_column(
        primary_key=True, default=uuid.uuid4, index=True
    )
    prompt: Mapped[str] = mapped_column(String)
    status: Mapped[str | None] = mapped_column(
        sa.Enum(
            "planning",
            "executing",
            "validating",
            "accepted",
            "rejected",
            "failed",
            name="sessionstatus",
        )
    )
    validation_logs: Mapped[dict | None] = mapped_column(JSON)


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
    skill_git_hash: Mapped[str | None] = mapped_column(String)
    template_versions: Mapped[dict | None] = mapped_column(JSON)
    metadata_vars: Mapped[dict | None] = mapped_column(JSON)

    # Agent state
    todo_list: Mapped[dict | None] = mapped_column(JSON)
    journal: Mapped[str | None] = mapped_column(String)
    plan: Mapped[str | None] = mapped_column(String)

    traces: Mapped[list["Trace"]] = relationship(
        back_populates="episode", cascade="all, delete-orphan"
    )
    assets: Mapped[list["Asset"]] = relationship(
        back_populates="episode", cascade="all, delete-orphan"
    )

    @property
    def validation_logs(self) -> list[str] | None:
        if self.metadata_vars:
            return self.metadata_vars.get("validation_logs")
        return None


class Trace(Base):
    __tablename__ = "traces"

    id: Mapped[int] = mapped_column(sa.BigInteger, primary_key=True, autoincrement=True)
    episode_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("episodes.id"))
    langfuse_trace_id: Mapped[str | None] = mapped_column(String)

    trace_type: Mapped[TraceType] = mapped_column(SQLEnum(TraceType))
    name: Mapped[str | None] = mapped_column(String)  # Tool name or stage
    content: Mapped[str | None] = mapped_column(
        String
    )  # Tool input/output or LLM content
    metadata_vars: Mapped[dict | None] = mapped_column(
        "metadata", JSON
    )  # Additional data
    feedback_score: Mapped[int | None] = mapped_column(Integer)  # 1 for up, 0 for down
    feedback_comment: Mapped[str | None] = mapped_column(String)

    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

    episode: Mapped["Episode"] = relationship(back_populates="traces")


class Asset(Base):
    __tablename__ = "assets"

    id: Mapped[int] = mapped_column(sa.BigInteger, primary_key=True, autoincrement=True)
    episode_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("episodes.id"))
    asset_type: Mapped[AssetType] = mapped_column(SQLEnum(AssetType))
    s3_path: Mapped[str] = mapped_column(String)
    content: Mapped[str | None] = mapped_column(String, nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)

    episode: Mapped["Episode"] = relationship(back_populates="assets")
