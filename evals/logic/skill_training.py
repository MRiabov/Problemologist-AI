from __future__ import annotations

import argparse
import asyncio
import json
import os
import subprocess
from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from evals.logic.codex_session_trace import (
    CodexSessionTraceArtifact,
    capture_latest_codex_session_artifacts,
    snapshot_workspace_state,
)
from evals.logic.codex_workspace import (
    WorkspaceVerificationResult,
    get_cli_provider,
    resolve_cli_home_root,
    resume_cli_exec,
    verify_workspace_for_agent,
)
from evals.logic.models import EvalDatasetItem
from evals.logic.runner_reporting import RunnerLogContext, _sanitize_readable_text
from evals.logic.runner_skill_loop import (
    CodexSkillLoopSummary as SkillLoopSummary,
)
from evals.logic.runner_skill_loop import (
    _load_workspace_simulation_result,
    _run_skill_loop,
    _skill_loop_events_path,
    _skill_loop_needed,
)
from shared.enums import AgentName
from shared.logging import configure_logging, get_logger
from shared.models.simulation import SimulationResult
from worker_light.utils.git import copy_tree, init_workspace_repo

ROOT = Path(__file__).resolve().parents[2]
SKILL_OVERLAY_ENV = "PROBLEMOLOGIST_SKILL_OVERLAY_ROOT"


class SkillTrainingSessionMetadata(BaseModel):
    model_config = ConfigDict(extra="allow")

    agent_name: str | None = None
    complexity_level: int | None = None
    provider_name: str | None = None
    task_id: str | None = None
    task: str | None = None
    session_id: str | None = None
    episode_id: str | None = None
    workspace_dir: str | None = None
    launch_return_code: int | None = None
    success: bool | None = None
    status: str | None = None
    verification_name: str | None = None
    verification_errors: list[str] = Field(default_factory=list)
    verification_details: dict[str, Any] = Field(default_factory=dict)
    codex_session_trace: dict[str, Any] | None = None
    codex_skill_loop: dict[str, Any] | None = None
    skill_loop_journal_path: str | None = None
    skill_loop_context_snapshot_path: str | None = None
    suggested_skills_dir: str | None = None
    suggested_skills_base_commit: str | None = None
    suggested_skills_branch: str | None = None
    skills_repo_root: str | None = None
    skills_repo_head_commit: str | None = None
    skills_repo_branch: str | None = None


class SkillTrainingSession(BaseModel):
    session_metadata_path: Path
    session_log_root: Path
    eval_log_key: str
    workspace_dir: Path
    agent_name: AgentName
    task_id: str
    session_id: str
    provider_name: str
    cli_runtime_root: Path
    launch_return_code: int | None = None
    verification_result: WorkspaceVerificationResult
    simulation_result: SimulationResult | None = None
    item: EvalDatasetItem
    skill_loop_journal_path: Path | None = None
    skill_loop_context_snapshot_path: Path | None = None
    suggested_skills_dir: Path | None = None
    suggested_skills_base_commit: str | None = None
    suggested_skills_branch: str | None = None
    skills_repo_root: Path | None = None
    skills_repo_head_commit: str | None = None
    skills_repo_branch: str | None = None


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a retained CLI-provider episode bundle and resume the same session "
            "for bounded skill training."
        )
    )
    parser.add_argument(
        "--session-metadata-path",
        type=Path,
        default=None,
        help=(
            "Direct path to session_metadata.json for the retained episode "
            "bundle. If omitted, pass --session-log-root and --eval-log-key."
        ),
    )
    parser.add_argument(
        "--session-log-root",
        type=Path,
        default=ROOT / "logs" / "evals" / "current" / "sessions",
        help=(
            "Root directory containing retained session bundles. Defaults to "
            "logs/evals/current/sessions."
        ),
    )
    parser.add_argument(
        "--eval-log-key",
        default=None,
        help="Retained session key under the session log root, for example ec-001.",
    )
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=None,
        help="Override the retained workspace directory from session metadata.",
    )
    parser.add_argument(
        "--session-id",
        default=None,
        help="Override the retained session id from session metadata.",
    )
    parser.add_argument(
        "--task-id",
        default=None,
        help="Override the task id from session metadata.",
    )
    parser.add_argument(
        "--agent",
        default=None,
        help=(
            "Override the agent name from session metadata, for example engineer_coder."
        ),
    )
    parser.add_argument(
        "--codex-runtime-root",
        type=Path,
        default=None,
        help=(
            "Override the CLI runtime root. Defaults to the parent of the "
            "retained session log root."
        ),
    )
    parser.add_argument(
        "--log-level",
        default=os.getenv("LOG_LEVEL", "INFO"),
        help="Logging level for the training replay process.",
    )
    return parser.parse_args()


def _default_session_metadata_path(
    *,
    session_log_root: Path,
    eval_log_key: str,
) -> Path:
    return session_log_root / eval_log_key / "session_metadata.json"


def _load_session_metadata(path: Path) -> SkillTrainingSessionMetadata:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"session metadata must be a JSON object: {path}")
    return SkillTrainingSessionMetadata.model_validate(raw)


def _resolve_agent_name(
    *,
    metadata: SkillTrainingSessionMetadata,
    override: str | None,
) -> AgentName:
    candidate = override or metadata.agent_name
    if not candidate:
        raise ValueError("agent name is missing from the retained session metadata")
    return AgentName(candidate)


def _resolve_task_id(
    *,
    metadata: SkillTrainingSessionMetadata,
    override: str | None,
    eval_log_key: str,
    session_id: str,
) -> str:
    candidate = override or metadata.task_id or metadata.episode_id or eval_log_key
    if candidate:
        return candidate
    return session_id


def _resolve_complexity_level(
    *,
    metadata: SkillTrainingSessionMetadata,
) -> int:
    if metadata.complexity_level is not None:
        return metadata.complexity_level
    # Legacy bundles may predate complexity-level persistence. Default to the
    # lowest valid value so retained-session replay remains loadable.
    return 0


def _resolve_session_id(
    *,
    metadata: SkillTrainingSessionMetadata,
    override: str | None,
    eval_log_key: str,
) -> str:
    candidate = override or metadata.session_id or metadata.episode_id
    if candidate:
        return candidate
    return eval_log_key


def _resolve_provider_name(
    *,
    metadata: SkillTrainingSessionMetadata,
    override: str | None,
) -> str:
    # Legacy retained bundles may not have recorded a provider yet. Treat those
    # as Codex sessions so replay stays anchored to the original backend instead
    # of inheriting whatever provider happens to be active in this process.
    candidate = override or metadata.provider_name or "codex"
    if not candidate:
        return "codex"
    return candidate


def _resolve_workspace_dir(
    *,
    metadata: SkillTrainingSessionMetadata,
    override: Path | None,
) -> Path:
    candidate = override or (
        Path(metadata.workspace_dir).expanduser() if metadata.workspace_dir else None
    )
    if candidate is not None:
        return candidate.expanduser().resolve()
    raise ValueError(
        "workspace_dir is missing from the retained session metadata and no "
        "--workspace-dir override was provided"
    )


def _resolve_launch_return_code(
    *,
    metadata: SkillTrainingSessionMetadata,
) -> int | None:
    if metadata.launch_return_code is not None:
        return metadata.launch_return_code
    if metadata.success is True:
        return 0
    if metadata.success is False:
        return 1
    return None


def _resolve_optional_snapshot_path(
    *,
    metadata_path: Path,
    raw_path: str | None,
    field_name: str,
) -> Path | None:
    if raw_path is None or not raw_path.strip():
        return None
    path = Path(raw_path).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(
            f"{field_name} referenced by {metadata_path} does not exist: {path}"
        )
    return path


def _git_output(repo_root: Path, *args: str) -> str | None:
    completed = subprocess.run(
        ["git", "-C", str(repo_root), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return None
    return completed.stdout.strip() or None


def _read_skill_repo_state(repo_root: Path) -> tuple[str | None, str | None]:
    if not (repo_root / ".git").exists():
        return None, None
    return (
        _git_output(repo_root, "rev-parse", "HEAD"),
        _git_output(repo_root, "rev-parse", "--abbrev-ref", "HEAD"),
    )


def _resolve_skill_repo_root(workspace_dir: Path) -> Path | None:
    workspace_dir = workspace_dir.expanduser().resolve()
    nested_skill_root = workspace_dir / ".agents" / "skills"
    legacy_skill_root = workspace_dir / "skills"
    if (nested_skill_root / ".git").exists():
        return nested_skill_root
    if (legacy_skill_root / ".git").exists():
        return legacy_skill_root
    if (workspace_dir / ".git").exists():
        return workspace_dir
    return None


def _overlay_has_skill_content(root: Path) -> bool:
    return any(
        path.is_file() and not any(part == ".git" for part in path.parts)
        for path in root.rglob("*")
    )


def _seed_skill_overlay_state(
    workspace_dir: Path,
) -> tuple[Path, Path | None, str | None, str | None]:
    overlay_dir = workspace_dir / "suggested_skills"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    os.environ[SKILL_OVERLAY_ENV] = overlay_dir.as_posix()
    repo_root = _resolve_skill_repo_root(workspace_dir)
    base_commit, branch = (
        _read_skill_repo_state(repo_root) if repo_root is not None else (None, None)
    )

    skill_snapshot_root = workspace_dir / ".agents" / "skills"
    legacy_skill_snapshot_root = workspace_dir / "skills"
    if skill_snapshot_root.exists() and not _overlay_has_skill_content(overlay_dir):
        copy_tree(skill_snapshot_root, overlay_dir)
    elif legacy_skill_snapshot_root.exists() and not _overlay_has_skill_content(
        overlay_dir
    ):
        copy_tree(legacy_skill_snapshot_root, overlay_dir)
    init_workspace_repo(overlay_dir)
    return overlay_dir, repo_root, base_commit, branch


def _update_session_metadata(
    *,
    path: Path,
    metadata: SkillTrainingSessionMetadata,
    overlay_dir: Path,
    repo_root: Path | None,
    base_commit: str | None,
    branch: str | None,
) -> None:
    payload = metadata.model_dump(mode="json")
    if metadata.complexity_level is None:
        payload.pop("complexity_level", None)
    if metadata.provider_name is None:
        payload.pop("provider_name", None)
    payload["suggested_skills_dir"] = overlay_dir.as_posix()
    payload["suggested_skills_base_commit"] = base_commit
    payload["suggested_skills_branch"] = branch
    payload["skills_repo_root"] = (
        repo_root.as_posix() if repo_root is not None else None
    )
    payload["skills_repo_head_commit"] = base_commit
    payload["skills_repo_branch"] = branch
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


async def load_skill_training_session(
    *,
    args: argparse.Namespace,
) -> SkillTrainingSession:
    if args.session_metadata_path is not None:
        session_metadata_path = args.session_metadata_path.expanduser().resolve()
        if not session_metadata_path.exists():
            raise FileNotFoundError(
                f"Session metadata not found: {session_metadata_path}"
            )
        session_log_root = session_metadata_path.parent.parent
        eval_log_key = session_metadata_path.parent.name
    else:
        if args.eval_log_key is None:
            raise ValueError(
                "Pass --session-metadata-path or both --session-log-root and "
                "--eval-log-key."
            )
        session_log_root = args.session_log_root.expanduser().resolve()
        eval_log_key = args.eval_log_key
        session_metadata_path = _default_session_metadata_path(
            session_log_root=session_log_root,
            eval_log_key=eval_log_key,
        )
        if not session_metadata_path.exists():
            raise FileNotFoundError(
                f"Session metadata not found: {session_metadata_path}"
            )

    metadata = _load_session_metadata(session_metadata_path)
    provider_name = _resolve_provider_name(metadata=metadata, override=None)
    provider = get_cli_provider(provider_name)
    workspace_dir = _resolve_workspace_dir(
        metadata=metadata,
        override=args.workspace_dir,
    )
    agent_name = _resolve_agent_name(metadata=metadata, override=args.agent)
    session_id = _resolve_session_id(
        metadata=metadata,
        override=args.session_id,
        eval_log_key=eval_log_key,
    )
    task_id = _resolve_task_id(
        metadata=metadata,
        override=args.task_id,
        eval_log_key=eval_log_key,
        session_id=session_id,
    )
    cli_runtime_root = (
        args.codex_runtime_root.expanduser().resolve()
        if args.codex_runtime_root is not None
        else session_log_root.parent.resolve()
    )
    (
        suggested_skills_dir,
        skills_repo_root,
        suggested_skills_base_commit,
        skills_repo_branch,
    ) = _seed_skill_overlay_state(workspace_dir)
    _update_session_metadata(
        path=session_metadata_path,
        metadata=metadata,
        overlay_dir=suggested_skills_dir,
        repo_root=skills_repo_root,
        base_commit=suggested_skills_base_commit,
        branch=skills_repo_branch,
    )

    verification_result = await verify_workspace_for_agent(
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        session_id=session_id,
        expected_decision=None,
    )
    simulation_result = _load_workspace_simulation_result(workspace_dir)
    skill_loop_journal_path = _resolve_optional_snapshot_path(
        metadata_path=session_metadata_path,
        raw_path=metadata.skill_loop_journal_path,
        field_name="skill_loop_journal_path",
    )
    skill_loop_context_snapshot_path = _resolve_optional_snapshot_path(
        metadata_path=session_metadata_path,
        raw_path=metadata.skill_loop_context_snapshot_path,
        field_name="skill_loop_context_snapshot_path",
    )

    return SkillTrainingSession(
        session_metadata_path=session_metadata_path,
        session_log_root=session_log_root,
        eval_log_key=eval_log_key,
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        task_id=task_id,
        session_id=session_id,
        cli_runtime_root=cli_runtime_root,
        provider_name=provider.provider_name,
        launch_return_code=_resolve_launch_return_code(metadata=metadata),
        verification_result=verification_result,
        simulation_result=simulation_result,
        item=EvalDatasetItem(
            id=task_id,
            task=metadata.task or f"Retained CLI-provider session {session_id}",
            complexity_level=_resolve_complexity_level(metadata=metadata),
        ),
        skill_loop_journal_path=skill_loop_journal_path,
        skill_loop_context_snapshot_path=skill_loop_context_snapshot_path,
        suggested_skills_dir=suggested_skills_dir,
        suggested_skills_base_commit=suggested_skills_base_commit,
        suggested_skills_branch=skills_repo_branch,
        skills_repo_root=skills_repo_root,
        skills_repo_head_commit=suggested_skills_base_commit,
        skills_repo_branch=skills_repo_branch,
    )


async def run_skill_training_session(
    *,
    session: SkillTrainingSession,
    log,
) -> tuple[SkillLoopSummary, CodexSessionTraceArtifact | None]:
    log_context = RunnerLogContext(root=ROOT, session_log_root=session.session_log_root)
    provider = get_cli_provider(session.provider_name)
    should_run, trigger_reason = _skill_loop_needed(
        agent_name=session.agent_name,
        launch_return_code=session.launch_return_code,
        verification_result=session.verification_result,
        simulation_result=session.simulation_result,
    )
    if not should_run:
        summary = SkillLoopSummary(
            enabled=True,
            triggered=False,
            trigger_reason=trigger_reason,
            simulation_success=(
                session.simulation_result.success
                if session.simulation_result is not None
                else None
            ),
            events_path=_skill_loop_events_path(session.workspace_dir).as_posix(),
            primary_turn=None,
        )
        return summary, None

    baseline_snapshot = snapshot_workspace_state(workspace_dir=session.workspace_dir)
    codex_trace_artifacts = await asyncio.to_thread(
        capture_latest_codex_session_artifacts,
        workspace_dir=session.workspace_dir,
        artifact_root=(
            session.session_log_root / session.eval_log_key / provider.provider_name
        ),
        baseline_snapshot=baseline_snapshot,
        launched_after_ns=None,
        sessions_root=resolve_cli_home_root(
            task_id=session.task_id,
            session_id=session.session_id,
            runtime_root=session.cli_runtime_root,
        )
        / provider.home_dir_name
        / "sessions",
    )
    if codex_trace_artifacts is None or not codex_trace_artifacts.session_id:
        raise FileNotFoundError(
            "missing CLI-provider session trace for the retained episode bundle"
        )

    summary, updated_trace = await _run_skill_loop(
        item=session.item,
        agent_name=session.agent_name,
        workspace_dir=session.workspace_dir,
        cli_runtime_root=session.cli_runtime_root,
        log_context=log_context,
        baseline_snapshot=baseline_snapshot,
        codex_trace_artifacts=codex_trace_artifacts,
        primary_session_id=session.session_id,
        launch_return_code=session.launch_return_code,
        verification_result=session.verification_result,
        log=log,
        provider_name=session.provider_name,
        deps={
            "capture_latest_codex_session_artifacts": (
                capture_latest_codex_session_artifacts
            ),
            "resume_cli_exec": resume_cli_exec,
            "resolve_cli_home_root": resolve_cli_home_root,
            "eval_log_key": session.eval_log_key,
        },
    )
    return summary, updated_trace


async def main() -> int:
    args = _parse_args()
    configure_logging("evals")
    log = get_logger(__name__)

    try:
        session = await load_skill_training_session(args=args)
        summary, _ = await run_skill_training_session(session=session, log=log)
    except Exception as exc:
        log.exception("skill_training_failed", error=_sanitize_readable_text(exc))
        return 1

    if summary.failure_reason:
        log.warning(
            "skill_training_completed_with_failure",
            trigger_reason=summary.trigger_reason,
            failure_reason=summary.failure_reason,
            events_path=summary.events_path,
        )
        return 1

    log.info(
        "skill_training_completed",
        triggered=summary.triggered,
        trigger_reason=summary.trigger_reason,
        events_path=summary.events_path,
    )
    return 0


def run_cli() -> None:
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("Skill training interrupted.", flush=True)


if __name__ == "__main__":
    run_cli()
