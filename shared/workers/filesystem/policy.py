import fnmatch
from pathlib import Path
from typing import Literal

from shared.agents.config import (
    AgentExecutionPolicy,
    VisualInspectionPolicy,
    load_agents_config,
)
from shared.enums import AgentName

PolicyAction = Literal["read", "write"]


class FilesystemPolicy:
    def __init__(self, config_path: str | Path | None = None):
        self.config = load_agents_config(config_path)

    @staticmethod
    def _normalize_virtual_path(path: str | Path) -> str:
        """Normalize workspace aliases to the canonical session-root path."""
        normalized = Path(path).as_posix()
        if normalized in {"/workspace", "workspace"}:
            normalized = "/"
        elif normalized.startswith("/workspace/"):
            normalized = "/" + normalized[len("/workspace/") :]
        elif normalized.startswith("workspace/"):
            normalized = normalized[len("workspace/") :]

        # Treat the worker-facing skill mount as an alias for the canonical
        # checked-in tree. This avoids requiring a repo-local `skills/`
        # directory while keeping the read policy stable.
        if normalized == "skills":
            normalized = ".agents/skills"
        elif normalized.startswith("skills/"):
            normalized = ".agents/skills/" + normalized[len("skills/") :]

        parts: list[str] = []
        escaped_root = False
        for part in normalized.lstrip("/").split("/"):
            if part in {"", "."}:
                continue
            if part == "..":
                if parts:
                    parts.pop()
                else:
                    escaped_root = True
                continue
            parts.append(part)

        if escaped_root:
            return "__ESCAPE_ROOT__"
        return "/".join(parts)

    def _match_path(self, path: str, patterns: list[str]) -> bool:
        """Check if path matches any of the gitignore-style glob patterns."""
        if not patterns:
            return False

        p_str = self._normalize_virtual_path(path)

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

                # `**/` should match zero or more directories so patterns like
                # `**/*.py` also allow root-level files such as `script.py`.
                regex_pat = re.escape(pat)
                regex_pat = regex_pat.replace(r"\*\*/", r"(?:.*/)?")
                regex_pat = regex_pat.replace(r"\*\*", ".*")
                regex_pat = regex_pat.replace(r"\*", "[^/]*")
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

        p_str = self._normalize_virtual_path(path)

        if action == "write" and p_str == "bug_report.md":
            if not self.config.bug_reports.enabled:
                return False
            agent_rules = self.config.agents.get(role)
            if agent_rules is None:
                return False
            if not agent_rules.write.allow:
                return False
            if self._match_path(p_str, agent_rules.write.deny):
                return False
            return True

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

    def get_visual_inspection_policy(
        self, agent_role: AgentName | str
    ) -> VisualInspectionPolicy:
        role = (
            agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        )
        agent_rules = self.config.agents.get(role)
        if agent_rules:
            return agent_rules.visual_inspection
        return self.config.defaults.visual_inspection
