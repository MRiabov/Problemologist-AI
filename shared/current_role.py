from __future__ import annotations

from pathlib import Path

from shared.enums import AgentName
from shared.script_contracts import CURRENT_ROLE_MANIFEST_PATH
from shared.workers.schema import CurrentRoleManifest


def current_role_manifest_path() -> Path:
    return CURRENT_ROLE_MANIFEST_PATH


def current_role_manifest_json(agent_name: AgentName | str) -> str:
    manifest = CurrentRoleManifest(agent_name=AgentName(str(agent_name)))
    return manifest.model_dump_json(indent=2)


def parse_current_role_manifest(content: str) -> CurrentRoleManifest:
    return CurrentRoleManifest.model_validate_json(content)


def current_role_agent_name_from_text(content: str) -> AgentName:
    return parse_current_role_manifest(content).agent_name


def current_role_agent_name(workspace_root: Path) -> AgentName:
    manifest_path = workspace_root / CURRENT_ROLE_MANIFEST_PATH
    if not manifest_path.exists():
        raise FileNotFoundError(
            f"Missing current-role manifest: {manifest_path.as_posix()}"
        )
    return current_role_agent_name_from_text(manifest_path.read_text(encoding="utf-8"))
