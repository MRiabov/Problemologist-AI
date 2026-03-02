import fnmatch
from pathlib import Path
from typing import Literal

import yaml

PolicyAction = Literal["read", "write"]


class FilesystemPolicy:
    # Maps internal agent names/node names to canonical policy roles
    ROLE_MAPPING = {
        "engineer_planner": "engineering_planner",
        "engineering_planner": "engineering_planner",
        "engineer_coder": "engineering_mechanical_coder",
        "cad_engineer": "engineering_mechanical_coder",
        "engineering_mechanical_coder": "engineering_mechanical_coder",
        "electronics_engineer": "engineering_electrical_coder",
        "engineering_electrical_coder": "engineering_electrical_coder",
        "benchmark_planner": "benchmark_planner",
        "benchmark_generator": "benchmark_cad_coder",
        "benchmark_cad_coder": "benchmark_cad_coder",
        "engineering_reviewer": "engineering_reviewer",
        "reviewer": "engineering_reviewer",
        "engineer_critic": "engineering_reviewer",
    }

    def __init__(self, config_path: str | Path):
        with open(config_path) as f:
            self.config = yaml.safe_load(f)
        self.defaults = self.config.get("defaults", {})
        self.agents = self.config.get("agents", {})

    def _match_path(self, path: str, patterns: list[str]) -> bool:
        """Check if path matches any of the gitignore-style glob patterns."""
        p_str = Path(path).as_posix().lstrip("/")

        for pattern in patterns:
            pat = pattern.lstrip("/")

            # 1. Exact match or wildcard match using fnmatch
            if fnmatch.fnmatch(p_str, pat):
                return True

            # 2. Directory match: if pattern is 'dir/', match 'dir/file'
            if pat.endswith("/"):
                if p_str.startswith(pat):
                    return True

            # 3. Recursive directory match: if pattern is 'dir/**', match 'dir/file' and 'dir/subdir/file'
            if pat.endswith("/**"):
                base = pat[:-3]
                if p_str == base or p_str.startswith(base + "/"):
                    return True

            # 4. Folder prefix (implied recursion in some systems): if pattern is 'dir', match 'dir/file'
            if "/" not in pat:  # simple folder name
                # If path starts with pat and a slash, or is exactly pat
                if p_str == pat or p_str.startswith(pat + "/"):
                    return True
            elif not any(c in pat for c in "*?[]"):  # path without wildcards
                if p_str == pat or p_str.startswith(pat + "/"):
                    return True

        return False

    def check_permission(
        self, agent_role: str, action: PolicyAction, path: str | Path
    ) -> bool:
        """
        Check if an agent role has permission for an action on a path.
        Precedence: deny > allow.
        Unmatched => deny.
        """
        # Normalize agent role
        role = self.ROLE_MAPPING.get(agent_role, agent_role)

        p_str = Path(path).as_posix().lstrip("/")

        # Get rules for agent, fallback to defaults
        agent_rules = self.agents.get(role, {})
        action_rules = agent_rules.get(action)

        if action_rules is None:
            # Fallback to defaults
            action_rules = self.defaults.get(action, {})

        allow_patterns = action_rules.get("allow", [])
        deny_patterns = action_rules.get("deny", [])

        # 1. Deny takes precedence
        if self._match_path(p_str, deny_patterns):
            return False

        # 2. Check allow
        if self._match_path(p_str, allow_patterns):
            return True

        # 3. Default deny
        return False
