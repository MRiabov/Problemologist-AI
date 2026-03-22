from __future__ import annotations

import os
from pathlib import Path

import pytest

from evals.logic.codex_session_trace import (
    discover_codex_session_file,
    render_codex_session_artifacts,
    snapshot_workspace_state,
)

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = ROOT / "tests/integration/fixtures/codex_session_trace_sample.jsonl"


def _write_session_copy(
    *,
    destination: Path,
    cwd: Path,
) -> Path:
    text = FIXTURE.read_text(encoding="utf-8").replace(
        "/tmp/codex-workspace",
        cwd.as_posix(),
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(text, encoding="utf-8")
    return destination


@pytest.mark.integration_p0
def test_codex_session_trace_capture_keeps_artifacts_separate(tmp_path: Path) -> None:
    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()

    script_path = workspace_dir / "script.py"
    notes_path = workspace_dir / "notes.md"
    stable_path = workspace_dir / "stable.txt"

    script_path.write_text('print("before")\n', encoding="utf-8")
    notes_path.write_text("remove me\n", encoding="utf-8")
    stable_path.write_text("stay the same\n", encoding="utf-8")

    baseline = snapshot_workspace_state(workspace_dir=workspace_dir)

    script_path.write_text('print("after")\n', encoding="utf-8")
    notes_path.unlink()
    (workspace_dir / "new.txt").write_text("new file\n", encoding="utf-8")

    sessions_root = tmp_path / ".codex" / "sessions"
    matching_session = _write_session_copy(
        destination=sessions_root / "2026" / "03" / "22" / "rollout-match.jsonl",
        cwd=workspace_dir,
    )
    other_session = _write_session_copy(
        destination=sessions_root / "2026" / "03" / "22" / "rollout-other.jsonl",
        cwd=tmp_path / "some-other-workspace",
    )

    older_ns = 1_000_000_000
    newer_ns = 2_000_000_000
    os.utime(matching_session, ns=(older_ns, older_ns))
    os.utime(other_session, ns=(newer_ns, newer_ns))

    discovered = discover_codex_session_file(
        workspace_dir=workspace_dir,
        sessions_root=sessions_root,
    )
    assert discovered == matching_session

    artifact_root = tmp_path / "artifacts" / "codex"
    artifact = render_codex_session_artifacts(
        session_file=matching_session,
        workspace_dir=workspace_dir,
        artifact_root=artifact_root,
        baseline_snapshot=baseline,
    )

    assert artifact.session_id == "019d0000-0000-7000-9000-000000000001"
    assert artifact.raw_trace_path is not None
    assert artifact.transcript_path is not None
    assert artifact.workspace_diff_path is not None
    assert artifact.summary_path is not None
    assert artifact.artifact_dir is not None
    assert artifact.artifact_dir.parent == artifact_root
    assert artifact.raw_trace_path.read_text(
        encoding="utf-8"
    ) == matching_session.read_text(encoding="utf-8")

    transcript = artifact.transcript_path.read_text(encoding="utf-8")
    assert "SESSION_META id=019d0000-0000-7000-9000-000000000001" in transcript
    assert (
        "MESSAGE role=assistant phase=commentary text=Inspecting the workspace"
        in transcript
    )
    assert "REASONING [redacted encrypted_content]" in transcript
    assert 'TOOL_CALL exec_command args={"cmd":"git status --short"' in transcript
    assert (
        'TOOL_OUTPUT call_id=call-1 output=Command: /bin/bash -lc "git status --short"'
        in transcript
    )
    assert "EVENT task_started" in transcript
    assert "EVENT agent_message" in transcript

    assert artifact.transcript_stats.session_meta_count == 1
    assert artifact.transcript_stats.turn_context_count == 1
    assert artifact.transcript_stats.event_count == 2
    assert artifact.transcript_stats.message_count == 3
    assert artifact.transcript_stats.reasoning_count == 1
    assert artifact.transcript_stats.redacted_reasoning_count == 1
    assert artifact.transcript_stats.tool_call_count == 1
    assert artifact.transcript_stats.tool_output_count == 1

    diff = artifact.workspace_diff
    assert diff is not None
    assert diff.workspace_dir == workspace_dir
    assert "script.py" in diff.changed_paths
    assert "notes.md" in diff.changed_paths
    assert "new.txt" in diff.changed_paths

    modified_paths = {entry.path: entry for entry in diff.modified}
    assert "script.py" in modified_paths
    assert modified_paths["script.py"].diff is not None
    assert '-print("before")' in modified_paths["script.py"].diff
    assert '+print("after")' in modified_paths["script.py"].diff

    added_paths = {entry.path for entry in diff.added}
    removed_paths = {entry.path for entry in diff.removed}
    assert "new.txt" in added_paths
    assert "notes.md" in removed_paths

    summary_text = artifact.summary_path.read_text(encoding="utf-8")
    assert '"tool_call_count": 1' in summary_text
    assert '"reasoning_count": 1' in summary_text
    assert '"workspace_diff_path":' in summary_text
