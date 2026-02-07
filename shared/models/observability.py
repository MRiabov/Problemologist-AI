from datetime import UTC, datetime
from enum import Enum
from typing import Any

from pydantic import BaseModel, Field, StrictFloat, StrictInt, StrictStr


class AssetType(str, Enum):
    VIDEO = "VIDEO"
    IMAGE = "IMAGE"
    LOG = "LOG"


class TraceEvent(BaseModel):
    trace_id: StrictStr = Field(..., description="Unique identifier for the trace")
    agent_id: StrictStr = Field(..., description="ID of the agent producing the event")
    input_tokens: StrictInt | None = Field(
        None, description="Number of input tokens used"
    )
    output_tokens: StrictInt | None = Field(
        None, description="Number of output tokens used"
    )
    latency_ms: StrictFloat | None = Field(None, description="Latency in milliseconds")
    content: Any = Field(..., description="Content of the event")
    timestamp: datetime = Field(
        default_factory=lambda: datetime.now(UTC), description="Timestamp of the event"
    )


class AssetRecord(BaseModel):
    asset_id: StrictStr = Field(..., description="Unique identifier for the asset")
    trace_id: StrictStr = Field(..., description="ID of the associated trace")
    s3_key: StrictStr = Field(..., description="Key of the asset in S3 storage")
    asset_type: AssetType = Field(..., description="Type of the asset")
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(UTC), description="Creation timestamp"
    )
