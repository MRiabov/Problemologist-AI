from __future__ import annotations

import contextlib
import difflib
import json
import os
import shutil
from hashlib import sha256
from pathlib import Path
from typing import Any, Literal

from pydantic import BaseModel, Field

_TEXT_NAMES = {
    "Dockerfile",
    "LICENSE",
    "Makefile",
    "README",
    "README.md",
}
_TEXT_SUFFIXES = {
    ".cfg",
    ".ini",
    ".json",
    ".md",
    ".py",
    ".sh",
    ".toml",
    ".txt",
    ".yaml",
    ".yml",
    ".xml",
}
_SKIP_DIR_NAMES = {
    ".git",
    "__pycache__",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
}


class WorkspaceFileSnapshot(BaseModel):
    path: str
    kind: Literal["text", "binary"]
    size_bytes: int
    sha256: str
    text: str | None = None


class WorkspaceSnapshot(BaseModel):
    workspace_dir: Path
    files: list[WorkspaceFileSnapshot] = Field(default_factory=list)


class WorkspaceDiffEntry(BaseModel):
    path: str
    kind: Literal["text", "binary"]
    change_type: Literal["added", "removed", "modified"]
    before_sha256: str | None = None
    after_sha256: str | None = None
    diff: str | None = None


class WorkspaceDiffSummary(BaseModel):
    workspace_dir: Path
    added: list[WorkspaceDiffEntry] = Field(default_factory=list)
    removed: list[WorkspaceDiffEntry] = Field(default_factory=list)
    modified: list[WorkspaceDiffEntry] = Field(default_factory=list)

    @property
    def changed_paths(self) -> list[str]:
        return [
            *[entry.path for entry in self.added],
            *[entry.path for entry in self.removed],
            *[entry.path for entry in self.modified],
        ]


class CodexTranscriptStats(BaseModel):
    session_meta_count: int = 0
    turn_context_count: int = 0
    event_count: int = 0
    message_count: int = 0
    user_message_count: int = 0
    developer_message_count: int = 0
    assistant_message_count: int = 0
    reasoning_count: int = 0
    redacted_reasoning_count: int = 0
    tool_call_count: int = 0
    tool_output_count: int = 0


class CodexSessionTraceArtifact(BaseModel):
    session_id: str | None = None
    session_file: Path | None = None
    artifact_dir: Path | None = None
    raw_trace_path: Path | None = None
    transcript_path: Path | None = None
    summary_path: Path | None = None
    workspace_diff_path: Path | None = None
    transcript_stats: CodexTranscriptStats = Field(default_factory=CodexTranscriptStats)
    workspace_diff: WorkspaceDiffSummary | None = None
    warnings: list[str] = Field(default_factory=list)


def _codex_sessions_root() -> Path:
    codex_home = os.environ.get("CODEX_HOME")
    if codex_home:
        return Path(codex_home).expanduser() / "sessions"
    return Path.home() / ".codex" / "sessions"


def _is_text_path(path: Path) -> bool:
    return path.name in _TEXT_NAMES or path.suffix.lower() in _TEXT_SUFFIXES


def _normalize_text(value: Any, *, limit: int | None = None) -> str:
    if value is None:
        return ""
    text = str(value).replace("\r\n", "\n").replace("\r", "\n").strip()
    if not text:
        return ""
    text = " ".join(part for part in text.split())
    if limit is not None and len(text) > limit:
        text = text[: limit - 3].rstrip() + "..."
    return text


def _compact_json(value: Any, *, limit: int = 280) -> str:
    if isinstance(value, str):
        with contextlib.suppress(json.JSONDecodeError):
            parsed = json.loads(value)
            return _normalize_text(
                json.dumps(parsed, sort_keys=True, separators=(",", ":")),
                limit=limit,
            )
        return _normalize_text(value, limit=limit)
    try:
        text = json.dumps(value, sort_keys=True, separators=(",", ":"))
    except Exception:
        text = _normalize_text(value)
    return _normalize_text(text, limit=limit)


def _sha256_text(content: str) -> str:
    return sha256(content.encode("utf-8")).hexdigest()


def _snapshot_file(path: Path, workspace_dir: Path) -> WorkspaceFileSnapshot | None:
    if any(
        part in _SKIP_DIR_NAMES for part in path.relative_to(workspace_dir).parts[:-1]
    ):
        return None

    if not path.is_file():
        return None

    with contextlib.suppress(OSError):
        raw = path.read_bytes()
        size_bytes = len(raw)
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            return WorkspaceFileSnapshot(
                path=path.relative_to(workspace_dir).as_posix(),
                kind="binary",
                size_bytes=size_bytes,
                sha256=sha256(raw).hexdigest(),
            )

        return WorkspaceFileSnapshot(
            path=path.relative_to(workspace_dir).as_posix(),
            kind="text",
            size_bytes=size_bytes,
            sha256=_sha256_text(text),
            text=text,
        )

    return None


def snapshot_workspace_state(*, workspace_dir: Path) -> WorkspaceSnapshot:
    workspace_dir = workspace_dir.expanduser().resolve()
    files: list[WorkspaceFileSnapshot] = []

    for path in sorted(p for p in workspace_dir.rglob("*") if p.is_file()):
        if any(part in _SKIP_DIR_NAMES for part in path.parts):
            continue
        snapshot = _snapshot_file(path, workspace_dir)
        if snapshot is not None:
            files.append(snapshot)

    return WorkspaceSnapshot(workspace_dir=workspace_dir, files=files)


def _workspace_snapshot_by_path(
    snapshot: WorkspaceSnapshot,
) -> dict[str, WorkspaceFileSnapshot]:
    return {entry.path: entry for entry in snapshot.files}


def diff_workspace_snapshots(
    *,
    before: WorkspaceSnapshot | None,
    after: WorkspaceSnapshot,
) -> WorkspaceDiffSummary:
    before_map = _workspace_snapshot_by_path(before) if before is not None else {}
    after_map = _workspace_snapshot_by_path(after)
    workspace_dir = after.workspace_dir

    added: list[WorkspaceDiffEntry] = []
    removed: list[WorkspaceDiffEntry] = []
    modified: list[WorkspaceDiffEntry] = []

    for path in sorted(after_map.keys() - before_map.keys()):
        entry = after_map[path]
        added.append(
            WorkspaceDiffEntry(
                path=path,
                kind=entry.kind,
                change_type="added",
                after_sha256=entry.sha256,
            )
        )

    for path in sorted(before_map.keys() - after_map.keys()):
        entry = before_map[path]
        removed.append(
            WorkspaceDiffEntry(
                path=path,
                kind=entry.kind,
                change_type="removed",
                before_sha256=entry.sha256,
            )
        )

    for path in sorted(before_map.keys() & after_map.keys()):
        before_entry = before_map[path]
        after_entry = after_map[path]
        if before_entry.sha256 == after_entry.sha256:
            continue
        diff_text: str | None = None
        if before_entry.kind == "text" and after_entry.kind == "text":
            before_lines = (before_entry.text or "").splitlines()
            after_lines = (after_entry.text or "").splitlines()
            diff_lines = difflib.unified_diff(
                before_lines,
                after_lines,
                fromfile=f"a/{path}",
                tofile=f"b/{path}",
                lineterm="",
            )
            diff_text = "\n".join(diff_lines)
            if len(diff_text) > 12000:
                diff_text = diff_text[:11997].rstrip() + "..."
        modified.append(
            WorkspaceDiffEntry(
                path=path,
                kind=after_entry.kind,
                change_type="modified",
                before_sha256=before_entry.sha256,
                after_sha256=after_entry.sha256,
                diff=diff_text,
            )
        )

    return WorkspaceDiffSummary(
        workspace_dir=workspace_dir,
        added=added,
        removed=removed,
        modified=modified,
    )


def _session_meta_from_file(session_file: Path) -> dict[str, Any] | None:
    try:
        with session_file.open(encoding="utf-8") as handle:
            first_line = handle.readline()
    except Exception:
        return None
    if not first_line.strip():
        return None

    with contextlib.suppress(json.JSONDecodeError):
        parsed = json.loads(first_line)
        if isinstance(parsed, dict):
            payload = parsed.get("payload")
            if isinstance(payload, dict) and parsed.get("type") == "session_meta":
                return payload
    return None


def discover_codex_session_file(
    *,
    workspace_dir: Path,
    sessions_root: Path | None = None,
    launched_after_ns: int | None = None,
) -> Path | None:
    root = (sessions_root or _codex_sessions_root()).expanduser()
    if not root.exists():
        return None

    workspace_dir = workspace_dir.expanduser().resolve()
    candidates: list[tuple[int, Path]] = []
    for path in root.rglob("rollout-*.jsonl"):
        if not path.is_file():
            continue
        with contextlib.suppress(OSError):
            stat = path.stat()
            if launched_after_ns is not None and stat.st_mtime_ns < launched_after_ns:
                continue
            candidates.append((stat.st_mtime_ns, path))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0], reverse=True)
    for _, candidate in candidates:
        meta = _session_meta_from_file(candidate)
        if not isinstance(meta, dict):
            continue
        cwd = meta.get("cwd")
        if cwd is None:
            continue
        if Path(str(cwd)).expanduser().resolve() == workspace_dir:
            return candidate

    return None


def _format_session_line(
    *,
    record: dict[str, Any],
    transcript_stats: CodexTranscriptStats,
) -> str | None:
    record_type = record.get("type")
    payload = record.get("payload")

    if record_type == "session_meta" and isinstance(payload, dict):
        transcript_stats.session_meta_count += 1
        return (
            "SESSION_META "
            f"id={_normalize_text(payload.get('id'), limit=40)} "
            f"cwd={_normalize_text(payload.get('cwd'), limit=120)} "
            f"model={_normalize_text(payload.get('model'), limit=60)} "
            f"originator={_normalize_text(payload.get('originator'), limit=40)}"
        )

    if record_type == "turn_context" and isinstance(payload, dict):
        transcript_stats.turn_context_count += 1
        return (
            "TURN_CONTEXT "
            f"turn_id={_normalize_text(payload.get('turn_id'), limit=40)} "
            f"cwd={_normalize_text(payload.get('cwd'), limit=120)} "
            f"model={_normalize_text(payload.get('model'), limit=60)} "
            f"effort={_normalize_text(payload.get('effort'), limit=20)}"
        )

    if record_type == "event_msg" and isinstance(payload, dict):
        transcript_stats.event_count += 1
        event_type = _normalize_text(payload.get("type"), limit=48)
        message = _normalize_text(payload.get("message"), limit=260)
        phase = _normalize_text(payload.get("phase"), limit=32)
        if message:
            return f"EVENT {event_type} phase={phase} message={message}".rstrip()
        return f"EVENT {event_type}"

    if record_type != "response_item" or not isinstance(payload, dict):
        return None

    item_type = payload.get("type")

    if item_type == "message":
        transcript_stats.message_count += 1
        role = _normalize_text(payload.get("role"), limit=20)
        phase = _normalize_text(payload.get("phase"), limit=20)
        content = payload.get("content")
        parts: list[str] = []
        if isinstance(content, list):
            for element in content:
                if not isinstance(element, dict):
                    continue
                element_type = element.get("type")
                if element_type in {"input_text", "output_text"}:
                    text = _normalize_text(element.get("text"), limit=320)
                    if text:
                        parts.append(text)
        if role == "user":
            transcript_stats.user_message_count += 1
        elif role == "developer":
            transcript_stats.developer_message_count += 1
        elif role == "assistant":
            transcript_stats.assistant_message_count += 1
        body = " | ".join(parts)
        if body:
            return f"MESSAGE role={role} phase={phase} text={body}".rstrip()
        return f"MESSAGE role={role} phase={phase}".rstrip()

    if item_type == "reasoning":
        transcript_stats.reasoning_count += 1
        content = payload.get("content")
        if isinstance(content, str) and content.strip():
            return f"REASONING {_normalize_text(content, limit=420)}"
        summary = payload.get("summary")
        if isinstance(summary, list) and summary:
            summary_text = " | ".join(
                _normalize_text(item, limit=120) for item in summary if item
            )
            if summary_text:
                return f"REASONING SUMMARY {summary_text}"
        transcript_stats.redacted_reasoning_count += 1
        if payload.get("encrypted_content"):
            return "REASONING [redacted encrypted_content]"
        return "REASONING [redacted]"

    if item_type == "function_call":
        transcript_stats.tool_call_count += 1
        name = _normalize_text(payload.get("name"), limit=60)
        arguments = payload.get("arguments")
        return f"TOOL_CALL {name} args={_compact_json(arguments)}"

    if item_type == "function_call_output":
        transcript_stats.tool_output_count += 1
        call_id = _normalize_text(payload.get("call_id"), limit=48)
        output = _normalize_text(payload.get("output"), limit=420)
        if output:
            return f"TOOL_OUTPUT call_id={call_id} output={output}"
        return f"TOOL_OUTPUT call_id={call_id}"

    return None


def render_codex_session_artifacts(
    *,
    session_file: Path,
    workspace_dir: Path,
    artifact_root: Path,
    baseline_snapshot: WorkspaceSnapshot | None = None,
) -> CodexSessionTraceArtifact:
    session_file = session_file.expanduser().resolve()
    workspace_dir = workspace_dir.expanduser().resolve()
    artifact_root = artifact_root.expanduser().resolve()

    lines: list[str] = []
    transcript_stats = CodexTranscriptStats()
    session_id: str | None = None

    with session_file.open(encoding="utf-8") as handle:
        for raw_line in handle:
            raw_line = raw_line.strip()
            if not raw_line:
                continue
            with contextlib.suppress(json.JSONDecodeError):
                record = json.loads(raw_line)
                if not isinstance(record, dict):
                    continue
                if session_id is None and record.get("type") == "session_meta":
                    payload = record.get("payload")
                    if isinstance(payload, dict):
                        session_id = (
                            _normalize_text(payload.get("id"), limit=80) or None
                        )
                formatted = _format_session_line(
                    record=record,
                    transcript_stats=transcript_stats,
                )
                if formatted:
                    lines.append(formatted)

    if session_id is None:
        session_id = session_file.stem

    artifact_dir = artifact_root / session_id
    artifact_dir.mkdir(parents=True, exist_ok=True)

    raw_trace_path = artifact_dir / "session.jsonl"
    shutil.copy2(session_file, raw_trace_path)

    transcript_path = artifact_dir / "transcript.log"
    transcript_path.write_text(
        "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
    )

    current_snapshot = snapshot_workspace_state(workspace_dir=workspace_dir)
    workspace_diff = diff_workspace_snapshots(
        before=baseline_snapshot,
        after=current_snapshot,
    )
    workspace_diff_path = artifact_dir / "workspace_diff.json"
    workspace_diff_path.write_text(
        workspace_diff.model_dump_json(indent=2),
        encoding="utf-8",
    )

    summary_path = artifact_dir / "summary.json"
    artifact = CodexSessionTraceArtifact(
        session_id=session_id,
        session_file=session_file,
        artifact_dir=artifact_dir,
        raw_trace_path=raw_trace_path,
        transcript_path=transcript_path,
        summary_path=summary_path,
        workspace_diff_path=workspace_diff_path,
        transcript_stats=transcript_stats,
        workspace_diff=workspace_diff,
    )
    summary_path.write_text(artifact.model_dump_json(indent=2), encoding="utf-8")
    return artifact


def capture_latest_codex_session_artifacts(
    *,
    workspace_dir: Path,
    artifact_root: Path,
    baseline_snapshot: WorkspaceSnapshot | None = None,
    launched_after_ns: int | None = None,
    sessions_root: Path | None = None,
) -> CodexSessionTraceArtifact | None:
    session_file = discover_codex_session_file(
        workspace_dir=workspace_dir,
        sessions_root=sessions_root,
        launched_after_ns=launched_after_ns,
    )
    if session_file is None:
        return None
    return render_codex_session_artifacts(
        session_file=session_file,
        workspace_dir=workspace_dir,
        artifact_root=artifact_root,
        baseline_snapshot=baseline_snapshot,
    )
