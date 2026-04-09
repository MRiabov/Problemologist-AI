from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from shared.current_role import current_role_manifest_json
from shared.enums import AgentName
from shared.workers.schema import BugReportArchiveManifest


def _run_persist_bug_reports(
    *,
    workspace_root: Path,
    session_id: str,
    episode_id: str,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            "scripts/persist_bug_reports.py",
            "--workspace-root",
            str(workspace_root),
            "--session-id",
            session_id,
            "--episode-id",
            episode_id,
        ],
        cwd=Path(__file__).resolve().parents[3],
        capture_output=True,
        text=True,
        check=False,
    )


@pytest.mark.integration
@pytest.mark.integration_p1
def test_persist_bug_reports_archives_report_with_metadata(tmp_path: Path):
    workspace_root = tmp_path / "workspace"
    workspace_root.mkdir()
    manifests_dir = workspace_root / ".manifests"
    manifests_dir.mkdir()
    (manifests_dir / "current_role.json").write_text(
        current_role_manifest_json(AgentName.ENGINEER_CODER),
        encoding="utf-8",
    )
    bug_report_path = workspace_root / "bug_report.md"
    bug_report_path.write_text(
        "\n".join(
            [
                "Role: engineer_coder",
                "Session: session-123",
                "Episode: episode-456",
                "Command: python solution_script.py",
                "Expected: validation should run",
                "Actual: workspace materialization failed",
                "Artifact: prompt.md",
            ]
        ),
        encoding="utf-8",
    )

    result = _run_persist_bug_reports(
        workspace_root=workspace_root,
        session_id="session-123",
        episode_id="episode-456",
    )

    assert result.returncode == 0, result.stderr
    manifest = BugReportArchiveManifest.model_validate_json(result.stdout)
    assert manifest.agent_name == AgentName.ENGINEER_CODER
    assert manifest.session_id == "session-123"
    assert manifest.episode_id == "episode-456"
    assert manifest.workspace_dir == workspace_root.resolve().as_posix()
    assert manifest.source_path == "bug_report.md"
    assert manifest.archive_root == (workspace_root / "logs/bug_reports").as_posix()
    assert manifest.archived_report_path.endswith("/bug_report.md")

    archived_report = Path(manifest.archived_report_path)
    assert archived_report.exists()
    assert archived_report.read_text(encoding="utf-8") == bug_report_path.read_text(
        encoding="utf-8"
    )

    archived_manifest_path = archived_report.parent / "bug_report_archive_manifest.json"
    assert archived_manifest_path.exists()
    archived_manifest = json.loads(archived_manifest_path.read_text(encoding="utf-8"))
    assert archived_manifest["agent_name"] == "engineer_coder"
    assert archived_manifest["source_path"] == "bug_report.md"
    assert archived_manifest["archived_report_path"] == manifest.archived_report_path


@pytest.mark.integration
@pytest.mark.integration_p1
def test_persist_bug_reports_fails_closed_when_report_is_missing(tmp_path: Path):
    workspace_root = tmp_path / "workspace"
    workspace_root.mkdir()
    manifests_dir = workspace_root / ".manifests"
    manifests_dir.mkdir()
    (manifests_dir / "current_role.json").write_text(
        current_role_manifest_json(AgentName.ENGINEER_CODER),
        encoding="utf-8",
    )

    result = _run_persist_bug_reports(
        workspace_root=workspace_root,
        session_id="session-123",
        episode_id="episode-456",
    )

    assert result.returncode != 0
    assert "bug report not found" in result.stderr
    assert not (workspace_root / "logs/bug_reports").exists()
