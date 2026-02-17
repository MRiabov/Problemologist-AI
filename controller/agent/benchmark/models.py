from uuid import UUID

from pydantic import BaseModel, Field, HttpUrl

from shared.enums import SessionStatus
from shared.simulation.schemas import (
    AssetMetadata,
    CustomObjectives,
    SimulatorBackendType,
)


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
    status: SessionStatus = SessionStatus.PLANNING
    backend: SimulatorBackendType = SimulatorBackendType.GENESIS
    validation_logs: list[str] = Field(default_factory=list)
    custom_objectives: CustomObjectives = Field(default_factory=CustomObjectives)
