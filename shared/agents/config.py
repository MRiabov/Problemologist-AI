from __future__ import annotations

from pathlib import Path

import structlog
import yaml
from pydantic import BaseModel, Field, field_validator, model_validator

from shared.enums import AgentName

logger = structlog.get_logger(__name__)


class PathPolicy(BaseModel):
    allow: list[str] = Field(default_factory=list)
    deny: list[str] = Field(default_factory=list)


class FilesystemPermissions(BaseModel):
    read: PathPolicy = Field(default_factory=PathPolicy)
    write: PathPolicy = Field(default_factory=PathPolicy)


class VisualInspectionPolicy(BaseModel):
    required: bool = False
    min_images: int = Field(default=1, ge=1)
    reminder_interval: int = Field(default=2, ge=1)


class AgentPolicy(BaseModel):
    filesystem_permissions: FilesystemPermissions = Field(
        default_factory=FilesystemPermissions
    )
    tools: list[str] | None = None
    visual_inspection: VisualInspectionPolicy = Field(
        default_factory=VisualInspectionPolicy
    )

    @model_validator(mode="before")
    @classmethod
    def _normalize_filesystem_permissions(
        cls, value: object
    ) -> dict[str, object] | object:
        if not isinstance(value, dict):
            return value

        has_legacy = "read" in value or "write" in value
        has_new = "filesystem_permissions" in value
        if has_legacy and has_new:
            raise ValueError(
                "AgentPolicy cannot define both legacy read/write and "
                "filesystem_permissions keys."
            )

        if not has_legacy:
            return value

        normalized = dict(value)
        normalized["filesystem_permissions"] = {
            "read": normalized.pop("read", {}),
            "write": normalized.pop("write", {}),
        }
        return normalized

    @property
    def read(self) -> PathPolicy:
        return self.filesystem_permissions.read

    @property
    def write(self) -> PathPolicy:
        return self.filesystem_permissions.write


class LLMPolicyConfig(BaseModel):
    max_reasoning_tokens: int = 16384
    context_compaction_threshold_tokens: int = 225000
    requests_per_minute: int = Field(default=40, ge=1)
    multimodal_model: str | None = None


class RenderPolicyConfig(BaseModel):
    rgb: bool = True
    depth: bool = True
    segmentation: bool = True


class AgentExecutionPolicy(BaseModel):
    timeout_seconds: int = Field(default=300, ge=1)
    max_turns: int = Field(default=150, ge=1)
    max_total_tokens: int = Field(default=400000, ge=1)
    native_tool_loop_max_iters: int = Field(default=8, ge=1)


class AgentExecutionConfig(BaseModel):
    defaults: AgentExecutionPolicy = Field(default_factory=AgentExecutionPolicy)
    agents: dict[str, AgentExecutionPolicy] = Field(default_factory=dict)

    @field_validator("agents", mode="before")
    @classmethod
    def _normalize_agent_keys(
        cls, value: object
    ) -> dict[str, AgentExecutionPolicy] | object:
        if not isinstance(value, dict):
            return value
        normalized: dict[str, AgentExecutionPolicy] = {}
        for raw_key, raw_policy in value.items():
            key = str(raw_key)
            try:
                key = AgentName(key).value
            except Exception:
                key = key.strip().lower()
            if isinstance(raw_policy, AgentExecutionPolicy):
                normalized[key] = raw_policy
            else:
                normalized[key] = AgentExecutionPolicy.model_validate(raw_policy)
        return normalized

    def get_policy(self, agent_role: AgentName | str) -> AgentExecutionPolicy:
        key = agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        policy = self.agents.get(key)
        if policy:
            return policy
        return self.defaults


class AgentsConfig(BaseModel):
    llm: LLMPolicyConfig = Field(default_factory=LLMPolicyConfig)
    render: RenderPolicyConfig = Field(default_factory=RenderPolicyConfig)
    execution: AgentExecutionConfig = Field(default_factory=AgentExecutionConfig)
    defaults: AgentPolicy = Field(default_factory=AgentPolicy)
    agents: dict[str, AgentPolicy] = Field(default_factory=dict)


# Backward-compatible alias while import sites are migrated.
FilesystemConfig = AgentsConfig


def resolve_agents_config_path() -> Path | None:
    candidates = (
        Path("config/agents_config.yaml"),
        Path(__file__).parents[2] / "config" / "agents_config.yaml",
        Path("/app/config/agents_config.yaml"),
    )
    return next((p for p in candidates if p.exists()), None)


def load_agents_config(config_path: str | Path | None = None) -> AgentsConfig:
    resolved_path = (
        Path(config_path) if config_path is not None else resolve_agents_config_path()
    )
    if resolved_path is None:
        return AgentsConfig()

    try:
        with resolved_path.open("r", encoding="utf-8") as handle:
            raw_data = yaml.safe_load(handle) or {}
        return AgentsConfig.model_validate(raw_data)
    except Exception as exc:
        logger.warning(
            "failed_to_load_agents_config",
            config_path=str(resolved_path),
            error=str(exc),
        )
        return AgentsConfig()
