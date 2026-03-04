import fnmatch
from pathlib import Path
from typing import Literal

import yaml

from shared.enums import AgentName

PolicyAction = Literal["read", "write"]


from pydantic import BaseModel, Field


class PathPolicy(BaseModel):
    allow: list[str] = Field(default_factory=list)
    deny: list[str] = Field(default_factory=list)


class AgentPolicy(BaseModel):
    read: PathPolicy = Field(default_factory=PathPolicy)
    write: PathPolicy = Field(default_factory=PathPolicy)


class FilesystemConfig(BaseModel):
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
            if pat.endswith("/"):
                if p_str.startswith(pat):
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
        # Normalize agent role to string value if it's an enum
        role = agent_role.value if isinstance(agent_role, AgentName) else agent_role

        # Mapping for legacy/node-specific names that don't match AgentName enum directly
        # but should share the same policy.
        LEGACY_MAPPING = {
            "engineering_mechanical_coder": AgentName.ENGINEER_CODER.value,
            "engineering_electrical_coder": AgentName.ELECTRONICS_ENGINEER.value,
            "benchmark_cad_coder": AgentName.BENCHMARK_CODER.value,
            "engineering_reviewer": AgentName.ENGINEER_REVIEWER.value,
            "reviewer": AgentName.ENGINEER_REVIEWER.value,
            "engineer_critic": AgentName.ENGINEER_REVIEWER.value,
            "planner": AgentName.ENGINEER_PLANNER.value,
            "coder": AgentName.ENGINEER_CODER.value,
        }
        role = LEGACY_MAPPING.get(role, role)

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
