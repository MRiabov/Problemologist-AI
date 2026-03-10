import uuid
from typing import Any

from pydantic import BaseModel, Field, StrictStr, field_validator

from shared.enums import AgentName
from shared.models.schemas import EpisodeMetadata


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")
    user_session_id: uuid.UUID | None = Field(
        None, description="UI conversation scope session ID."
    )
    metadata_vars: EpisodeMetadata | None = Field(
        None, description="Additional metadata for the episode."
    )
    skill_git_hash: str | None = Field(
        None, description="Git hash of the skills used for this run."
    )
    agent_name: AgentName = Field(
        AgentName.ENGINEER_CODER, description="The name of the agent to run."
    )

    @field_validator("task", "session_id", "skill_git_hash")
    @classmethod
    def strip_null_bytes(cls, v: str | None) -> str | None:
        if v is None:
            return None
        return v.replace("\u0000", "")

    @field_validator("metadata_vars", mode="before")
    @classmethod
    def strip_null_bytes_in_metadata(cls, v: Any) -> Any:
        if v is None:
            return None

        def _clean(value):
            if isinstance(value, str):
                return value.replace("\u0000", "")
            if isinstance(value, dict):
                return {
                    (_clean(k) if isinstance(k, str) else k): _clean(val)
                    for k, val in value.items()
                }
            if isinstance(value, list):
                return [_clean(item) for item in value]
            return value

        return _clean(v)
