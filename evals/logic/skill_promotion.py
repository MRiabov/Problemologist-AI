from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
from datetime import UTC, datetime
from difflib import SequenceMatcher
from pathlib import Path
from typing import Literal

import structlog
from pydantic import BaseModel, ConfigDict, Field

from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SkillPromotionEvent
from worker_light.utils.git import (
    commit_all,
    repo_head_state,
    repo_is_clean,
)

ROOT = Path(__file__).resolve().parents[2]
logger = structlog.get_logger(__name__)


class SkillPromotionResult(BaseModel):
    ok: bool
    outcome: Literal["published", "conflict", "escalated", "rejected"]
    codex_session_id: str | None = None
    overlay_root: str
    canonical_root: str
    approved_base_commit: str | None = None
    target_branch: str | None = None
    merge_strategy: str | None = None
    promotion_commit: str | None = None
    changed_paths: list[str] = Field(default_factory=list)
    conflicting_skill_paths: list[str] = Field(default_factory=list)
    promotion_record_path: str | None = None
    reason: str | None = None


class SkillPromotionSessionMetadata(BaseModel):
    model_config = ConfigDict(extra="allow")

    session_id: str | None = None
    episode_id: str | None = None
    workspace_dir: str | None = None
    suggested_skills_dir: str | None = None
    suggested_skills_base_commit: str | None = None
    skills_repo_root: str | None = None
    skills_repo_head_commit: str | None = None
    skills_repo_branch: str | None = None


def _default_overlay_root(workspace_dir: Path) -> Path:
    return workspace_dir / "suggested_skills"


def _default_canonical_root(workspace_dir: Path) -> Path:
    return workspace_dir / "skills"


def _record_promotion_result(
    workspace_dir: Path,
    result: SkillPromotionResult,
) -> Path:
    record_dir = workspace_dir / "logs" / "skill_loop"
    record_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    session_fragment = (
        "".join(
            ch if ch.isalnum() or ch in {"-", "_"} else "-"
            for ch in result.codex_session_id
        )
        if result.codex_session_id
        else "anonymous"
    )
    record_path = record_dir / f"skill_promotion-{session_fragment}-{timestamp}.json"
    record_path.write_text(
        json.dumps(result.model_dump(mode="json"), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return record_path


def _finalize_promotion_result(
    *,
    workspace_dir: Path,
    result: SkillPromotionResult,
    session_id: str | None,
) -> SkillPromotionResult:
    result.promotion_record_path = _record_promotion_result(
        workspace_dir, result
    ).as_posix()
    try:
        asyncio.run(_record_promotion_event(session_id=session_id, result=result))
    except RuntimeError:
        logger.warning(
            "skill_promotion_event_record_skipped_running_loop",
            session_id=session_id or result.codex_session_id or "",
        )
    return result


def _read_session_metadata(metadata_path: Path) -> SkillPromotionSessionMetadata:
    raw = json.loads(metadata_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"session metadata must be a JSON object: {metadata_path}")
    return SkillPromotionSessionMetadata.model_validate(raw)


def _resolve_path_candidate(
    *,
    metadata: SkillPromotionSessionMetadata | None,
    override: Path | None,
    field_name: str,
) -> Path | None:
    if override is not None:
        return override.expanduser().resolve()
    if metadata is None:
        return None
    raw_value = getattr(metadata, field_name, None)
    if raw_value:
        return Path(raw_value).expanduser().resolve()
    return None


def _overlay_changed_paths(overlay_root: Path) -> list[str]:
    changed_paths: list[str] = []
    for path in sorted(p for p in overlay_root.rglob("*") if p.is_file()):
        if any(part == ".git" for part in path.parts):
            continue
        changed_paths.append(path.relative_to(overlay_root).as_posix())
    return changed_paths


def _overlay_deleted_paths(overlay_root: Path) -> list[str]:
    if not (overlay_root / ".git").exists():
        return []
    try:
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(overlay_root),
                "diff",
                "--name-only",
                "--diff-filter=D",
                "HEAD",
                "--",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        return []
    if completed.returncode != 0:
        return []
    return [line.strip() for line in completed.stdout.splitlines() if line.strip()]


def _repo_changed_paths_since(repo_root: Path, base_commit: str) -> list[str]:
    try:
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(repo_root),
                "diff",
                "--name-only",
                f"{base_commit}..HEAD",
                "--",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        return []
    if completed.returncode != 0:
        return []
    return [line.strip() for line in completed.stdout.splitlines() if line.strip()]


def _read_repo_file_at_commit(
    *,
    repo_root: Path,
    commit: str,
    rel_path: str,
) -> str | None:
    try:
        completed = subprocess.run(
            [
                "git",
                "-C",
                str(repo_root),
                "show",
                f"{commit}:{rel_path}",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout


def _text_hunks(
    base_lines: list[str], other_lines: list[str]
) -> list[tuple[int, int, list[str]]]:
    return [
        (i1, i2, other_lines[j1:j2])
        for tag, i1, i2, j1, j2 in SequenceMatcher(
            a=base_lines, b=other_lines
        ).get_opcodes()
        if tag != "equal"
    ]


def _merge_text_file(
    *,
    base_text: str,
    current_text: str,
    overlay_text: str,
) -> str | None:
    base_lines = base_text.splitlines(keepends=True)
    current_lines = current_text.splitlines(keepends=True)
    overlay_lines = overlay_text.splitlines(keepends=True)

    current_hunks = _text_hunks(base_lines, current_lines)
    overlay_hunks = _text_hunks(base_lines, overlay_lines)

    for current_hunk in current_hunks:
        for overlay_hunk in overlay_hunks:
            current_start, current_end, current_replacement = current_hunk
            overlay_start, overlay_end, overlay_replacement = overlay_hunk
            if (
                current_start == current_end
                and overlay_start == overlay_end
                and current_start == overlay_start
            ):
                if current_replacement != overlay_replacement:
                    return None
                continue
            if current_start < overlay_end and overlay_start < current_end:
                if (
                    current_start == overlay_start
                    and current_end == overlay_end
                    and current_replacement == overlay_replacement
                ):
                    continue
                return None

    merged_hunks = sorted(
        [*current_hunks, *overlay_hunks], key=lambda item: (item[0], item[1], item[2])
    )
    merged_lines: list[str] = []
    cursor = 0
    seen_hunks: set[tuple[int, int, tuple[str, ...]]] = set()
    for start, end, replacement in merged_hunks:
        hunk_key = (start, end, tuple(replacement))
        if hunk_key in seen_hunks:
            continue
        seen_hunks.add(hunk_key)
        if start < cursor:
            return None
        merged_lines.extend(base_lines[cursor:start])
        merged_lines.extend(replacement)
        cursor = max(cursor, end)
    merged_lines.extend(base_lines[cursor:])
    return "".join(merged_lines)


def _plan_overlay_publication(
    *,
    canonical_root: Path,
    overlay_root: Path,
    approved_base_commit: str | None,
) -> tuple[list[str], dict[str, bytes], list[str]]:
    if not approved_base_commit:
        return [], {}, []
    current_state = repo_head_state(canonical_root)
    current_head = current_state.get("head_commit")
    if not current_head:
        return [], {}, []

    overlay_changed = set(_overlay_changed_paths(overlay_root))
    overlay_deleted = set(_overlay_deleted_paths(overlay_root))
    if not overlay_changed and not overlay_deleted:
        return [], {}, []

    canonical_changed = set(
        _repo_changed_paths_since(canonical_root, approved_base_commit)
    )
    planned_updates: dict[str, bytes] = {}
    conflicts: list[str] = []

    for rel_path in sorted({*overlay_changed, *overlay_deleted}):
        if rel_path in overlay_deleted:
            if rel_path in canonical_changed:
                conflicts.append(rel_path)
            continue

        overlay_path = overlay_root / rel_path
        canonical_path = canonical_root / rel_path
        overlay_bytes = overlay_path.read_bytes()
        current_bytes = canonical_path.read_bytes() if canonical_path.exists() else None

        if rel_path in canonical_changed:
            if overlay_path.suffix.lower() not in {
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
            }:
                if current_bytes != overlay_bytes:
                    conflicts.append(rel_path)
                continue

            base_text = _read_repo_file_at_commit(
                repo_root=canonical_root,
                commit=approved_base_commit,
                rel_path=rel_path,
            )
            if base_text is None:
                conflicts.append(rel_path)
                continue

            current_text = (
                current_bytes.decode("utf-8") if current_bytes is not None else ""
            )
            overlay_text = overlay_bytes.decode("utf-8")
            merged_text = _merge_text_file(
                base_text=base_text,
                current_text=current_text,
                overlay_text=overlay_text,
            )
            if merged_text is None:
                conflicts.append(rel_path)
                continue

            merged_bytes = merged_text.encode("utf-8")
            if current_bytes == merged_bytes:
                continue
            planned_updates[rel_path] = merged_bytes
            continue

        if current_bytes == overlay_bytes:
            continue
        planned_updates[rel_path] = overlay_bytes

    return conflicts, planned_updates, sorted(overlay_deleted)


def _apply_planned_overlay_updates(
    *,
    canonical_root: Path,
    planned_updates: dict[str, bytes],
) -> None:
    for rel_path, content in sorted(planned_updates.items()):
        dst_path = canonical_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_bytes(content)


async def _record_promotion_event(
    *,
    session_id: str | None,
    result: SkillPromotionResult,
) -> None:
    event = SkillPromotionEvent(
        episode_id=session_id or result.codex_session_id,
        user_session_id=session_id or result.codex_session_id,
        agent_id="skill_agent",
        codex_session_id=result.codex_session_id,
        active_overlay_path=result.overlay_root,
        approved_base_commit=result.approved_base_commit,
        target_repo="skills",
        target_branch=result.target_branch,
        merge_strategy=result.merge_strategy,
        outcome=result.outcome,
        promotion_commit=result.promotion_commit,
        promotion_record_path=result.promotion_record_path,
        conflicting_skill_paths=result.conflicting_skill_paths,
        reason=result.reason,
    )
    try:
        await record_worker_events(
            episode_id=session_id or result.codex_session_id or "skill_promotion",
            events=[event],
        )
    except Exception:
        logger.warning(
            "skill_promotion_event_record_failed",
            session_id=session_id or result.codex_session_id or "",
        )


def promote_skill_overlay(
    *,
    workspace_dir: Path,
    overlay_root: Path | None = None,
    canonical_root: Path | None = None,
    session_id: str | None = None,
    approved_base_commit: str | None = None,
    commit_message: str | None = None,
) -> SkillPromotionResult:
    overlay_root = (
        (overlay_root or _default_overlay_root(workspace_dir)).expanduser().resolve()
    )
    canonical_root = (
        (canonical_root or _default_canonical_root(workspace_dir))
        .expanduser()
        .resolve()
    )

    if not overlay_root.exists():
        result = SkillPromotionResult(
            ok=False,
            outcome="rejected",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            reason="suggested_skills overlay not found",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )
    if not canonical_root.exists():
        result = SkillPromotionResult(
            ok=False,
            outcome="rejected",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            reason="canonical skills repo not found",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )
    if not repo_is_clean(canonical_root):
        state = repo_head_state(canonical_root)
        result = SkillPromotionResult(
            ok=False,
            outcome="conflict",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            approved_base_commit=state.get("head_commit"),
            target_branch=state.get("branch"),
            merge_strategy="blocked-by-dirty-repo",
            conflicting_skill_paths=[],
            reason="canonical skills repo has uncommitted changes",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )

    base_state = repo_head_state(canonical_root)
    effective_base_commit = approved_base_commit or base_state.get("head_commit")
    conflicting_skill_paths, planned_updates, deleted_paths = _plan_overlay_publication(
        canonical_root=canonical_root,
        overlay_root=overlay_root,
        approved_base_commit=effective_base_commit,
    )
    if conflicting_skill_paths:
        result = SkillPromotionResult(
            ok=False,
            outcome="conflict",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            approved_base_commit=effective_base_commit,
            target_branch=base_state.get("branch"),
            merge_strategy="base-commit-overlap",
            conflicting_skill_paths=conflicting_skill_paths,
            reason="canonical skills repo advanced with overlapping skill changes",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )

    changed_paths = sorted({*planned_updates, *deleted_paths})
    if not changed_paths:
        result = SkillPromotionResult(
            ok=False,
            outcome="rejected",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            approved_base_commit=effective_base_commit,
            target_branch=base_state.get("branch"),
            merge_strategy="copy-overwrite",
            reason="overlay contained no changes to promote",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )

    _apply_planned_overlay_updates(
        canonical_root=canonical_root,
        planned_updates=planned_updates,
    )
    for rel_path in deleted_paths:
        target_path = canonical_root / rel_path
        if target_path.exists():
            target_path.unlink()

    message = commit_message or (
        "Promote suggested skills"
        + (f" for session {session_id}" if session_id else "")
    )
    promotion_commit = commit_all(canonical_root, message)
    if promotion_commit is None:
        result = SkillPromotionResult(
            ok=False,
            outcome="escalated",
            codex_session_id=session_id,
            overlay_root=overlay_root.as_posix(),
            canonical_root=canonical_root.as_posix(),
            approved_base_commit=base_state.get("head_commit"),
            target_branch=base_state.get("branch"),
            merge_strategy="copy-overwrite",
            changed_paths=changed_paths,
            reason="failed to commit promoted skill overlay",
        )
        return _finalize_promotion_result(
            workspace_dir=workspace_dir, result=result, session_id=session_id
        )

    published_state = repo_head_state(canonical_root)
    result = SkillPromotionResult(
        ok=True,
        outcome="published",
        codex_session_id=session_id,
        overlay_root=overlay_root.as_posix(),
        canonical_root=canonical_root.as_posix(),
        approved_base_commit=effective_base_commit,
        target_branch=published_state.get("branch"),
        merge_strategy="copy-overwrite",
        promotion_commit=promotion_commit,
        changed_paths=changed_paths,
    )
    return _finalize_promotion_result(
        workspace_dir=workspace_dir, result=result, session_id=session_id
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Promote a session-local suggested_skills overlay into the "
            "canonical skills repo."
        )
    )
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=ROOT,
        help="Workspace root containing suggested_skills/ and skills/.",
    )
    parser.add_argument(
        "--overlay-root",
        type=Path,
        default=None,
        help="Override the suggested_skills overlay root.",
    )
    parser.add_argument(
        "--canonical-root",
        type=Path,
        default=None,
        help="Override the canonical skills repo root.",
    )
    parser.add_argument(
        "--session-id",
        default=None,
        help="Optional session identifier used in the promotion commit message.",
    )
    parser.add_argument(
        "--approved-base-commit",
        default=None,
        help="Approved canonical base commit for the session overlay.",
    )
    parser.add_argument(
        "--session-metadata-path",
        type=Path,
        default=None,
        help=(
            "Optional retained session metadata JSON used to infer session "
            "and base-commit metadata."
        ),
    )
    parser.add_argument(
        "--commit-message",
        default=None,
        help="Override the git commit message used for publication.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    metadata = (
        _read_session_metadata(args.session_metadata_path.expanduser().resolve())
        if args.session_metadata_path is not None
        else None
    )
    workspace_dir = (
        _resolve_path_candidate(
            metadata=metadata,
            override=args.workspace_dir,
            field_name="workspace_dir",
        )
        or ROOT
    )
    overlay_root = _resolve_path_candidate(
        metadata=metadata,
        override=args.overlay_root,
        field_name="suggested_skills_dir",
    )
    canonical_root = _resolve_path_candidate(
        metadata=metadata,
        override=args.canonical_root,
        field_name="skills_repo_root",
    ) or _default_canonical_root(workspace_dir)
    session_id = (
        args.session_id
        or (metadata.session_id if metadata is not None else None)
        or (metadata.episode_id if metadata is not None else None)
    )
    approved_base_commit = (
        args.approved_base_commit
        or (metadata.suggested_skills_base_commit if metadata is not None else None)
        or (metadata.skills_repo_head_commit if metadata is not None else None)
    )
    result = promote_skill_overlay(
        workspace_dir=workspace_dir.expanduser().resolve(),
        overlay_root=overlay_root,
        canonical_root=canonical_root,
        session_id=session_id,
        approved_base_commit=approved_base_commit,
        commit_message=args.commit_message,
    )
    print(json.dumps(result.model_dump(mode="json"), indent=2, sort_keys=True))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
