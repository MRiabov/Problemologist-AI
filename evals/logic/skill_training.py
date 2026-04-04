from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
from typing import Any

from pydantic import BaseModel, Field

from evals.logic.codex_session_trace import (
    CodexSessionTraceArtifact,
    capture_latest_codex_session_artifacts,
    snapshot_workspace_state,
)
from evals.logic.codex_workspace import (
    WorkspaceVerificationResult,
    resolve_codex_home_root,
    resume_codex_exec,
    verify_workspace_for_agent,
)
from evals.logic.models import EvalDatasetItem
from evals.logic.runner_reporting import RunnerLogContext, _sanitize_readable_text
from evals.logic.runner_skill_loop import (
    CodexSkillLoopSummary,
    _codex_skill_loop_events_path,
    _codex_skill_loop_needed,
    _load_workspace_simulation_result,
    _run_codex_skill_loop,
)
from shared.enums import AgentName
from shared.logging import configure_logging, get_logger
from shared.models.simulation import SimulationResult

ROOT = Path(__file__).resolve().parents[2]


class SkillTrainingSessionMetadata(BaseModel):
    agent_name: str | None = None
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


class SkillTrainingSession(BaseModel):
    session_metadata_path: Path
    session_log_root: Path
    eval_log_key: str
    workspace_dir: Path
    agent_name: AgentName
    task_id: str
    session_id: str
    codex_runtime_root: Path
    launch_return_code: int | None = None
    verification_result: WorkspaceVerificationResult
    simulation_result: SimulationResult | None = None
    item: EvalDatasetItem


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a retained Codex episode bundle and resume the same session "
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
        help="Override the Codex session id from session metadata.",
    )
    parser.add_argument(
        "--task-id",
        default=None,
        help="Override the task id from session metadata.",
    )
    parser.add_argument(
        "--agent",
        default=None,
        help="Override the agent name from session metadata, for example engineer_coder.",
    )
    parser.add_argument(
        "--codex-runtime-root",
        type=Path,
        default=None,
        help=(
            "Override the Codex runtime root. Defaults to the parent of the "
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
                "Pass --session-metadata-path or both --session-log-root and --eval-log-key."
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
    codex_runtime_root = (
        args.codex_runtime_root.expanduser().resolve()
        if args.codex_runtime_root is not None
        else session_log_root.parent.resolve()
    )

    verification_result = await verify_workspace_for_agent(
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        session_id=session_id,
        expected_decision=None,
    )
    simulation_result = _load_workspace_simulation_result(workspace_dir)

    return SkillTrainingSession(
        session_metadata_path=session_metadata_path,
        session_log_root=session_log_root,
        eval_log_key=eval_log_key,
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        task_id=task_id,
        session_id=session_id,
        codex_runtime_root=codex_runtime_root,
        launch_return_code=_resolve_launch_return_code(metadata=metadata),
        verification_result=verification_result,
        simulation_result=simulation_result,
        item=EvalDatasetItem(
            id=task_id,
            task=metadata.task or f"Retained Codex session {session_id}",
        ),
    )


async def run_skill_training_session(
    *,
    session: SkillTrainingSession,
    log,
) -> tuple[CodexSkillLoopSummary, CodexSessionTraceArtifact | None]:
    log_context = RunnerLogContext(root=ROOT, session_log_root=session.session_log_root)
    should_run, trigger_reason = _codex_skill_loop_needed(
        agent_name=session.agent_name,
        launch_return_code=session.launch_return_code,
        verification_result=session.verification_result,
        simulation_result=session.simulation_result,
    )
    if not should_run:
        summary = CodexSkillLoopSummary(
            enabled=True,
            triggered=False,
            trigger_reason=trigger_reason,
            simulation_success=(
                session.simulation_result.success
                if session.simulation_result is not None
                else None
            ),
            events_path=_codex_skill_loop_events_path(session.workspace_dir).as_posix(),
            primary_turn=None,
        )
        return summary, None

    baseline_snapshot = snapshot_workspace_state(workspace_dir=session.workspace_dir)
    codex_trace_artifacts = await asyncio.to_thread(
        capture_latest_codex_session_artifacts,
        workspace_dir=session.workspace_dir,
        artifact_root=session.session_log_root / session.eval_log_key / "codex",
        baseline_snapshot=baseline_snapshot,
        launched_after_ns=None,
        sessions_root=resolve_codex_home_root(
            task_id=session.task_id,
            session_id=session.session_id,
            runtime_root=session.codex_runtime_root,
        )
        / ".codex"
        / "sessions",
    )
    if codex_trace_artifacts is None or not codex_trace_artifacts.session_id:
        raise FileNotFoundError(
            "missing Codex session trace for the retained episode bundle"
        )

    summary, updated_trace = await _run_codex_skill_loop(
        item=session.item,
        agent_name=session.agent_name,
        workspace_dir=session.workspace_dir,
        codex_runtime_root=session.codex_runtime_root,
        log_context=log_context,
        baseline_snapshot=baseline_snapshot,
        codex_trace_artifacts=codex_trace_artifacts,
        primary_session_id=session.session_id,
        launch_return_code=session.launch_return_code,
        verification_result=session.verification_result,
        log=log,
        deps={
            "capture_latest_codex_session_artifacts": capture_latest_codex_session_artifacts,
            "resume_codex_exec": resume_codex_exec,
            "resolve_codex_home_root": resolve_codex_home_root,
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
