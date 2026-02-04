from datetime import datetime
from typing import Any
from pydantic import BaseModel, Field


class EpisodeSummary(BaseModel):
    id: str
    timestamp: datetime
    name: str


class DashStep(BaseModel):
    index: int
    type: str
    agent_role: str | None
    content: str | None
    tool_name: str | None
    tool_input: str | None
    tool_output: str | None
    metadata: dict[str, Any]
    artifacts: list[str] = Field(default_factory=list)


class DashEpisode(BaseModel):
    id: str
    name: str
    steps: list[DashStep]
