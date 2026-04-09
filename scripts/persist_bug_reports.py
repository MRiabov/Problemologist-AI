#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from shared.current_role import current_role_agent_name  # noqa: E402
from shared.enums import AgentName  # noqa: E402
from shared.workers.schema import BugReportArchiveManifest  # noqa: E402


def _env_value(*names: str) -> str | None:
    for name in names:
        value = os.getenv(name, "").strip()
        if value:
            return value
    return None


def _sanitize_component(value: str) -> str:
    cleaned = []
    for char in value.strip():
        if char.isalnum() or char in {"-", "_", "."}:
            cleaned.append(char)
        else:
            cleaned.append("_")
    result = "".join(cleaned).strip("._")
    return result or "unknown"


def _resolve_relative_path(base: Path, candidate: Path) -> Path:
    if candidate.is_absolute():
        return candidate
    return base / candidate


def _hash_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=("Archive bug_report.md into a run-scoped logs/bug_reports/ tree.")
    )
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=Path.cwd(),
        help="Workspace root that contains bug_report.md and .manifests/current_role.json.",
    )
    parser.add_argument(
        "--bug-report-path",
        type=Path,
        default=Path("bug_report.md"),
        help="Path to the workspace bug report, relative to the workspace root unless absolute.",
    )
    parser.add_argument(
        "--archive-root",
        type=Path,
        default=Path("logs/bug_reports"),
        help="Archive root, relative to the workspace root unless absolute.",
    )
    parser.add_argument(
        "--session-id",
        type=str,
        default=_env_value(
            "SESSION_ID", "USER_SESSION_ID", "PROBLEMOLOGIST_SESSION_ID"
        ),
        help="Session identifier to record in the archive manifest.",
    )
    parser.add_argument(
        "--episode-id",
        type=str,
        default=_env_value("EPISODE_ID", "PROBLEMOLOGIST_EPISODE_ID"),
        help="Episode identifier to record in the archive manifest.",
    )
    parser.add_argument(
        "--agent-name",
        type=str,
        default=_env_value("AGENT_NAME", "PROBLEMOLOGIST_AGENT_NAME"),
        help="Override the agent role stored in .manifests/current_role.json.",
    )
    return parser.parse_args()


def archive_bug_report(
    *,
    workspace_root: Path,
    bug_report_path: Path,
    archive_root: Path,
    session_id: str | None,
    episode_id: str | None,
    agent_name: str | None,
) -> BugReportArchiveManifest:
    resolved_workspace_root = workspace_root.expanduser().resolve()
    resolved_bug_report_path = _resolve_relative_path(
        resolved_workspace_root, bug_report_path
    )
    if not resolved_bug_report_path.exists():
        raise FileNotFoundError(
            f"bug report not found at {resolved_bug_report_path.as_posix()}"
        )

    report_bytes = resolved_bug_report_path.read_bytes()
    report_text = report_bytes.decode("utf-8").strip()
    if not report_text:
        raise ValueError(
            f"bug report at {resolved_bug_report_path.as_posix()} is empty"
        )

    resolved_agent_name = (
        AgentName(str(agent_name))
        if agent_name is not None
        else current_role_agent_name(resolved_workspace_root)
    )

    resolved_archive_root = _resolve_relative_path(
        resolved_workspace_root, archive_root
    )
    session_component = _sanitize_component(session_id or "session-unknown")
    episode_component = _sanitize_component(episode_id or "episode-unknown")
    agent_component = _sanitize_component(resolved_agent_name.value)
    archive_dir = (
        resolved_archive_root / session_component / episode_component / agent_component
    )
    archive_dir.mkdir(parents=True, exist_ok=True)

    archived_report_path = archive_dir / "bug_report.md"
    archived_report_path.write_bytes(report_bytes)

    manifest = BugReportArchiveManifest(
        archived_at="",
        agent_name=resolved_agent_name,
        session_id=session_id,
        episode_id=episode_id,
        workspace_dir=resolved_workspace_root.as_posix(),
        source_path=resolved_bug_report_path.relative_to(
            resolved_workspace_root
        ).as_posix(),
        archive_root=resolved_archive_root.as_posix(),
        archive_dir=archive_dir.as_posix(),
        archived_report_path=archived_report_path.as_posix(),
        source_sha256=_hash_bytes(report_bytes),
        bug_report_size_bytes=len(report_bytes),
    )

    # Store the timestamp in ISO-like form without importing datetime twice.
    import datetime as _datetime

    manifest.archived_at = _datetime.datetime.now().isoformat()

    manifest_path = archive_dir / "bug_report_archive_manifest.json"
    manifest_path.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")
    return manifest


def main() -> int:
    args = _parse_args()
    manifest = archive_bug_report(
        workspace_root=args.workspace_root,
        bug_report_path=args.bug_report_path,
        archive_root=args.archive_root,
        session_id=args.session_id,
        episode_id=args.episode_id,
        agent_name=args.agent_name,
    )
    print(manifest.model_dump_json(indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
