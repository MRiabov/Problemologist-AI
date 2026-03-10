import uuid
from datetime import datetime
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, StrictStr, field_validator

from shared.enums import (
    AssetType,
    EpisodeStatus,
    ResponseStatus,
    ReviewDecision,
    TraceType,
)
from shared.models.schemas import EpisodeMetadata, TraceMetadata
from shared.models.steerability import CodeReference, GeometricSelection
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)


class StandardResponse(BaseModel):
    status: ResponseStatus
    message: str


class BenchmarkGenerateResponse(StandardResponse):
    session_id: uuid.UUID
    episode_id: uuid.UUID


class BenchmarkConfirmResponse(StandardResponse):
    pass


class AgentRunResponse(StandardResponse):
    episode_id: uuid.UUID


class TraceResponse(BaseModel):
    id: int
    user_session_id: uuid.UUID | None = None
    langfuse_trace_id: str | None
    simulation_run_id: str | None = None
    cots_query_id: str | None = None
    review_id: str | None = None
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata_vars: TraceMetadata | None = None
    feedback_score: int | None = None
    feedback_comment: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)

    @property
    def metadata(self) -> TraceMetadata | None:
        """Backward-compatible alias for clients using `metadata`."""
        return self.metadata_vars


class AssetResponse(BaseModel):
    id: int
    user_session_id: uuid.UUID | None = None
    asset_type: AssetType
    s3_path: str
    content: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)


class BenchmarkObjectivesResponse(StandardResponse):
    objectives: dict


class EpisodeResponse(BaseModel):
    id: uuid.UUID
    user_session_id: uuid.UUID | None = None
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    template_versions: dict | None = None
    metadata_vars: EpisodeMetadata | None = None
    todo_list: dict | None = None
    journal: str | None = None
    plan: str | None = None
    validation_logs: list[str] | None = None
    last_trace_id: int | None = None
    traces: list[TraceResponse] = []
    assets: list[AssetResponse] = []

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)

    @property
    def metadata(self) -> EpisodeMetadata | None:
        """Backward-compatible alias for clients using `metadata`."""
        return self.metadata_vars


class ArtifactEntry(BaseModel):
    """A single artifact entry from the /artifacts/{session_id} endpoint."""

    path: str
    size: int | None = None
    content: str | None = None


class EpisodeCreateResponse(BaseModel):
    """Response from the /test/episodes endpoint (used in integration tests)."""

    __test__ = False
    episode_id: uuid.UUID


class TestEpisodeCreateResponse(EpisodeCreateResponse):
    """Backward-compatible alias for episode create response."""

    __test__ = False


class EpisodeListItem(BaseModel):
    """A lightweight episode entry from the GET /episodes/ list endpoint."""

    id: uuid.UUID
    user_session_id: uuid.UUID | None = None
    task: str
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    metadata_vars: EpisodeMetadata | None = None
    journal: str | None = None
    plan: str | None = None
    todo_list: dict | None = None
    last_trace_id: int | None = None

    model_config = ConfigDict(from_attributes=True)


class ReviewResponse(BaseModel):
    """Response from posting a review decision to /episodes/{id}/review."""

    status: ResponseStatus
    decision: ReviewDecision


class SteeringQueueEntry(BaseModel):
    """A single entry in the steering prompt queue."""

    text: str
    selections: list[GeometricSelection] = []
    mentions: list[str] = []
    code_references: list[CodeReference] = []


class CotsSearchItem(BaseModel):
    """A single COTS part returned from search."""

    part_id: str
    name: str
    category: str
    manufacturer: str
    price: float
    source: str
    weight_g: float
    metadata: dict[str, Any] = Field(alias="metadata_vars", default_factory=dict)

    model_config = ConfigDict(populate_by_name=True)


class CotsMetadataResponse(BaseModel):
    """Metadata for the current COTS catalog."""

    catalog_version: str
    bd_warehouse_commit: str
    generated_at: str | None = None


class OpenAPISchema(BaseModel):
    """Minimal OpenAPI schema for integration testing contracts."""

    paths: dict[str, dict[str, Any]] = Field(default_factory=dict)

    model_config = ConfigDict(extra="allow")


# --- Consolidated Request Models ---


class BenchmarkGenerateRequest(BaseModel):
    prompt: str
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None
    seed_id: str | None = None
    seed_dataset: str | None = None
    generation_kind: str | None = None
    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)

    @field_validator("prompt")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


class ConfirmRequest(BaseModel):
    comment: str | None = None
    additional_turns: int = Field(
        default=0,
        ge=0,
        description="Optional extra turns to grant before resuming execution.",
    )


class UpdateObjectivesRequest(BaseModel):
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None


class RunSimulationRequest(BaseModel):
    session_id: str
    compound_json: str = "{}"
    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)


class FeedbackRequest(BaseModel):
    score: int  # 1 for up, 0 for down
    comment: str | None = None

    @field_validator("comment")
    @classmethod
    def strip_null_bytes(cls, v: str | None) -> str | None:
        if v is not None:
            return v.replace("\u0000", "")
        return v


class ReviewRequest(BaseModel):
    review_content: str

    @field_validator("review_content")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


class MessageRequest(BaseModel):
    message: StrictStr
    additional_turns: int = Field(
        default=0,
        ge=0,
        description="Optional extra turns to grant before continuing the episode.",
    )
    metadata_vars: EpisodeMetadata | None = Field(
        None, description="Additional metadata for the message."
    )

    @field_validator("message")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")
