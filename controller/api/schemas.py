import uuid
from datetime import datetime

from pydantic import BaseModel, ConfigDict, Field, StrictStr

from shared.enums import AssetType, EpisodeStatus, ResponseStatus, TraceType


class StandardResponse(BaseModel):
    status: ResponseStatus
    message: str


class BenchmarkGenerateResponse(StandardResponse):
    session_id: uuid.UUID


class BenchmarkConfirmResponse(StandardResponse):
    pass


class AgentRunResponse(StandardResponse):
    episode_id: uuid.UUID


class TraceResponse(BaseModel):
    id: int
    langfuse_trace_id: str | None
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata: dict | None = Field(None, alias="metadata_vars")
    feedback_score: int | None = None
    feedback_comment: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)


class AssetResponse(BaseModel):
    id: int
    asset_type: AssetType
    s3_path: str
    content: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)


class BenchmarkObjectivesResponse(StandardResponse):
    objectives: dict


class EpisodeResponse(BaseModel):
    id: uuid.UUID
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    template_versions: dict | None = None
    metadata: dict | None = Field(None, alias="metadata_vars")
    todo_list: dict | None = None
    journal: str | None = None
    plan: str | None = None
    validation_logs: list[str] | None = None
    traces: list[TraceResponse] = []
    assets: list[AssetResponse] = []

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)


class ArtifactEntry(BaseModel):
    """A single artifact entry from the /artifacts/{session_id} endpoint."""

    path: str
    size: int | None = None
    content: str | None = None


class TestEpisodeCreateResponse(BaseModel):
    """Response from the /test/episodes endpoint (used in integration tests)."""

    episode_id: uuid.UUID


class EpisodeListItem(BaseModel):
    """A lightweight episode entry from the GET /episodes/ list endpoint."""

    id: uuid.UUID
    task: str
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    metadata_vars: dict | None = None
    journal: str | None = None
    plan: str | None = None
    todo_list: dict | None = None

    model_config = ConfigDict(from_attributes=True)


class ReviewResponse(BaseModel):
    """Response from posting a review decision to /episodes/{id}/review."""

    status: str
    decision: str


class SteeringQueueEntry(BaseModel):
    """A single entry in the steering prompt queue."""

    text: str
    selections: list[dict] = []
    mentions: list[str] = []
