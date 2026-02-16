import uuid
from datetime import datetime
from pydantic import BaseModel, ConfigDict, Field, StrictStr
from shared.enums import AssetType, EpisodeStatus, TraceType


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


class EpisodeResponse(BaseModel):
    id: uuid.UUID
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    template_versions: dict | None = None
    metadata_vars: dict | None = None
    todo_list: dict | None = None
    journal: str | None = None
    plan: str | None = None
    validation_logs: list[str] | None = None
    traces: list[TraceResponse] = []
    assets: list[AssetResponse] = []

    model_config = ConfigDict(from_attributes=True)
