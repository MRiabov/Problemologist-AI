import fnmatch
from pathlib import Path
from typing import Literal

import yaml

from shared.enums import AgentName

PolicyAction = Literal["read", "write"]


from pydantic import BaseModel, Field, field_validator, model_validator


class PathPolicy(BaseModel):
    allow: list[str] = Field(default_factory=list)
    deny: list[str] = Field(default_factory=list)


class FilesystemPermissions(BaseModel):
    read: PathPolicy = Field(default_factory=PathPolicy)
    write: PathPolicy = Field(default_factory=PathPolicy)


class AgentPolicy(BaseModel):
    filesystem_permissions: FilesystemPermissions = Field(
        default_factory=FilesystemPermissions
    )
    tools: list[str] | None = None

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


class AgentExecutionPolicy(BaseModel):
    timeout_seconds: int = Field(default=300, ge=1)
    max_turns: int = Field(default=150, ge=1)
    max_total_tokens: int = Field(default=400000, ge=1)


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


class FilesystemConfig(BaseModel):
    llm: LLMPolicyConfig = Field(default_factory=LLMPolicyConfig)
    execution: AgentExecutionConfig = Field(default_factory=AgentExecutionConfig)
    defaults: AgentPolicy = Field(default_factory=AgentPolicy)
    agents: dict[str, AgentPolicy] = Field(default_factory=dict)


class FilesystemPolicy:
    def __init__(self, config_path: str | Path):
        with open(config_path) as f:
            data = yaml.safe_load(f)
            self.config = FilesystemConfig(**data)

    def _match_path(self, path: str, patterns: list[str]) -> bool:
        """Check if path matches any of the gitignore-style glob patterns."""
        if not patterns:
            return False

        p_str = Path(path).as_posix().lstrip("/")

        for pattern in patterns:
            pat = pattern.lstrip("/")

            # 1. Exact match
            if p_str == pat:
                return True

            # 2. Directory match: if pattern is 'dir/', match 'dir/file'
            if pat.endswith("/") and p_str.startswith(pat):
                return True

            # 3. Recursive directory match: if pattern is 'dir/**', match 'dir/file'
            if pat.endswith("/**"):
                base = pat[:-3]
                if p_str == base or p_str.startswith(base + "/"):
                    return True

            # 4. Wildcard match using fnmatch (handles * and ? correctly)
            if "**" in pat:
                import re

                regex_pat = (
                    re.escape(pat).replace(r"\*\*", ".*").replace(r"\*", "[^/]*")
                )
                if re.match(f"^{regex_pat}$", p_str):
                    return True
            elif fnmatch.fnmatch(p_str, pat):
                return True

            # 5. Folder prefix: if pattern is 'dir', match 'dir/file'
            if "/" not in pat:
                if p_str == pat or p_str.startswith(pat + "/"):
                    return True

        return False

    def check_permission(
        self, agent_role: str | AgentName, action: PolicyAction, path: str | Path
    ) -> bool:
        """
        Check if an agent role has permission for an action on a path.
        Precedence: deny > allow.
        Unmatched => deny.
        """
        # Strictly enforce AgentName enum
        role_enum = AgentName(agent_role) if isinstance(agent_role, str) else agent_role

        role = role_enum.value

        p_str = Path(path).as_posix().lstrip("/")

        # Get rules for agent, fallback to defaults
        agent_rules = self.config.agents.get(role)

        if agent_rules:
            action_rules = getattr(agent_rules, action)
        else:
            # Fallback to defaults
            action_rules = getattr(self.config.defaults, action)

        allow_patterns = action_rules.allow
        deny_patterns = action_rules.deny

        # 1. Deny takes precedence
        if self._match_path(p_str, deny_patterns):
            return False

        # 2. Check allow
        if self._match_path(p_str, allow_patterns):
            return True

        # 3. Default deny
        return False

    def get_execution_policy(self, agent_role: AgentName | str) -> AgentExecutionPolicy:
        role = (
            agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        )
        return self.config.execution.get_policy(role)

    def get_allowed_tools(self, agent_role: AgentName | str) -> set[str] | None:
        """
        Return allowed tool names for an agent role.
        - None means "no explicit tool policy" (caller may allow all defaults).
        - Empty set means "no tools allowed".
        """
        role = (
            agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        )
        agent_rules = self.config.agents.get(role)
        tools = agent_rules.tools if agent_rules else None
        if tools is None:
            return None
        return {str(t).strip() for t in tools if str(t).strip()}
