from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class FileListEntry(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    path: str | None = None
    is_dir: bool = False


class MessageContentBlock(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    text: str | None = None
    content: str | None = None


class NativeToolCallFunction(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    name: str | None = None
    arguments: str | None = None


class NativeToolCall(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    id: str | None = None
    type: str | None = None
    name: str | None = None
    tool_name: str | None = None
    arguments: str | None = None
    tool_input: str | None = None
    provider_specific_fields: dict[str, Any] | None = None
    function: NativeToolCallFunction | None = None


class NativeProviderMessage(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    content: Any | None = None
    tool_calls: list[Any] = Field(default_factory=list)
    provider_specific_fields: dict[str, Any] | None = None
    reasoning_content: str | None = None


class ProviderChoice(BaseModel):
    model_config = ConfigDict(from_attributes=True, extra="ignore")

    message: Any | None = None


class ProviderResponseEnvelope(BaseModel):
    model_config = ConfigDict(
        from_attributes=True, populate_by_name=True, extra="ignore"
    )

    response_ms: float | int | None = None
    hidden_params: dict[str, Any] | None = Field(default=None, alias="_hidden_params")
    choices: list[ProviderChoice] = Field(default_factory=list)
