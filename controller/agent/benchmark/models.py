from enum import StrEnum
from uuid import UUID

from pydantic import BaseModel, Field, HttpUrl

from shared.simulation.schemas import (
    AssetMetadata,
    CustomObjectives,
    SimulatorBackendType,
)


class SessionStatus(StrEnum):
    planning = "planning"
    planned = "planned"
    executing = "executing"
    validating = "validating"
    accepted = "accepted"
    rejected = "rejected"
    failed = "failed"


class BenchmarkAsset(BaseModel):
    benchmark_id: UUID
    mjcf_url: HttpUrl
    build123d_url: HttpUrl  # Original source code
    preview_bundle_url: HttpUrl  # ZIP of 24 multi-view images
    # Links to specific seeded variations
    random_variants: list[UUID] = Field(default_factory=list)
    difficulty_score: float = 0.0
    metadata: AssetMetadata = Field(default_factory=AssetMetadata)


class GenerationSession(BaseModel):
    session_id: UUID
    prompt: str
    status: SessionStatus = SessionStatus.planning
    backend: SimulatorBackendType = SimulatorBackendType.MUJOCO
    validation_logs: list[str] = Field(default_factory=list)
    custom_objectives: CustomObjectives = Field(default_factory=CustomObjectives)
