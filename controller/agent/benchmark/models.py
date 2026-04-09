from typing import Any
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, HttpUrl

from shared.enums import SessionStatus
from shared.simulation.schemas import (
    AssetMetadata,
    CustomObjectives,
    SimulatorBackendType,
    get_default_simulator_backend,
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


class BenchmarkItem(BaseModel):
    task: str | None = None
    prompt: str | None = None
    context: str | None = None
    expected_criteria: str | list[str] | None = None
    objectives: dict[str, Any] = Field(default_factory=dict)
    goals: str | None = None
    constraints: dict[str, Any] | None = None

    model_config = ConfigDict(extra="allow")


class BenchmarkExampleInputs(BaseModel):
    task: str | None = None
    prompt: str | None = None
    context: str | None = None
    expected_criteria: str | list[str] | None = None
    objectives: dict[str, Any] = Field(default_factory=dict)
    goals: str | None = None
    constraints: dict[str, Any] | None = None

    model_config = ConfigDict(extra="forbid")


class GenerationSession(BaseModel):
    session_id: UUID
    prompt: str
    status: SessionStatus = SessionStatus.PLANNING
    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)
    validation_logs: list[str] = Field(default_factory=list)
    custom_objectives: CustomObjectives = Field(default_factory=CustomObjectives)
