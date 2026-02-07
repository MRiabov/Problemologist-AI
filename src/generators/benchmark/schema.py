from sqlalchemy import JSON, UUID, Column, Enum, Float, String
from sqlalchemy.orm import declarative_base

from .models import SessionStatus

Base = declarative_base()


class BenchmarkAssetModel(Base):
    __tablename__ = "benchmark_assets"

    benchmark_id = Column(UUID, primary_key=True, index=True)
    mjcf_url = Column(String, nullable=False)
    build123d_url = Column(String, nullable=False)
    preview_bundle_url = Column(String, nullable=False)
    random_variants = Column(JSON, default=list)  # List of UUIDs
    difficulty_score = Column(Float, default=0.0)
    meta_data = Column(JSON, default=dict)


class GenerationSessionModel(Base):
    __tablename__ = "generation_sessions"

    session_id = Column(UUID, primary_key=True, index=True)
    prompt = Column(String, nullable=False)
    status = Column(Enum(SessionStatus), default=SessionStatus.planning)
    validation_logs = Column(JSON, default=list)
