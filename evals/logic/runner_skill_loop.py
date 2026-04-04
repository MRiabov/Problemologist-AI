from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

from pydantic import BaseModel

from controller.prompts import load_prompts
from evals.logic.codex_session_trace import CodexSessionTraceArtifact
from evals.logic.codex_workspace import (
    CodexExecRunResult,
    WorkspaceVerificationResult,
    is_coder_agent,
)
from evals.logic.codex_workspace import (
    resolve_codex_home_root as _resolve_codex_home_root,
)
from evals.logic.codex_workspace import (
    resume_codex_exec as _resume_codex_exec,
)
from evals.logic.models import EvalDatasetItem
from evals.logic.runner_reporting import (
    RunnerLogContext,
    _append_jsonl_record,
    _append_readable_log_line,
    _sanitize_readable_text,
)
from shared.agents.config import load_agents_config
from shared.enums import AgentName
from shared.models.simulation import SimulationResult
from shared.observability.schemas import SkillSelfReflectionEvent, SkillUpdateEvent


class CodexSkillLoopTurn(BaseModel):
    stage: str
    session_id: str | None = None
    return_code: int | None = None
    timed_out: bool = False
    prompt_path: str | None = None
    output_path: str | None = None
    simulation_success: bool | None = None
    verification_success: bool | None = None
    failure_reason: str | None = None


class CodexSkillLoopSummary(BaseModel):
    enabled: bool = False
    triggered: bool = False
    trigger_reason: str | None = None
    failure_reason: str | None = None
    simulation_success: bool | None = None
    events_path: str | None = None
    event_count: int = 0
    primary_turn: CodexSkillLoopTurn | None = None
    self_analysis_turn: CodexSkillLoopTurn | None = None
    skill_update_turn: CodexSkillLoopTurn | None = None


def _load_codex_skill_loop_prompt(prompt_key: str) -> str:
    codex_prompts = load_prompts().get("codex", {})
    skill_loop_prompts = codex_prompts.get("skill_loop", {})
    prompt = skill_loop_prompts.get(prompt_key)
    if isinstance(prompt, str) and prompt.strip():
        return prompt.strip()

    if prompt_key == "self_analysis":
        return (
            "Continue the same Codex session on a failed or stalled run.\n"
            "Analyze the workspace using `journal.md`, `prompt.md`, the session\n"
            "trace, `validation_results.json`, and `simulation_result.json` if it\n"
            "exists. Write a concise postmortem into `journal.md` with the failure\n"
            "mode, the key blocked attempt, and the reusable skills or procedures\n"
            "worth adding. Do not edit canonical `skills/` yet."
        )
    if prompt_key == "skill_update":
        return (
            "Continue the same Codex session.\n"
            "Using the postmortem and workspace artifacts, draft narrow skill\n"
            "updates into `suggested_skills/` as reviewable markdown files. Keep\n"
            "the edits incremental and avoid overwriting canonical `skills/`.\n"
            "If no skill update is warranted, record that in `journal.md` and stop."
        )
    raise KeyError(f"Unknown Codex skill loop prompt: {prompt_key}")


def _load_workspace_simulation_result(workspace_dir: Path) -> SimulationResult | None:
    simulation_result_path = workspace_dir / "simulation_result.json"
    if not simulation_result_path.exists():
        return None
    try:
        return SimulationResult.model_validate_json(
            simulation_result_path.read_text(encoding="utf-8")
        )
    except Exception:
        return None


def _codex_skill_loop_timeout_seconds(agent_name: AgentName) -> int:
    agent_config = load_agents_config().execution.agents.get(agent_name.value)
    if agent_config is None:
        return 120
    primary_timeout = int(agent_config.timeout_seconds)
    return max(90, min(300, primary_timeout // 4))


def _codex_skill_loop_needed(
    *,
    agent_name: AgentName,
    launch_return_code: int | None,
    verification_result: WorkspaceVerificationResult | None,
    simulation_result: SimulationResult | None,
) -> tuple[bool, str]:
    if not is_coder_agent(agent_name):
        return False, "skill loop only runs for coder roles"

    if launch_return_code is None:
        return True, "codex launch did not report a return code"

    if launch_return_code != 0:
        return True, f"codex exited with code {launch_return_code}"

    if verification_result is None:
        return True, "workspace verification did not run"

    if not verification_result.success:
        return True, "; ".join(
            verification_result.errors
        ) or verification_result.verification_name

    if simulation_result is None:
        return True, "simulation_result.json missing"

    if not simulation_result.success:
        return True, simulation_result.summary

    return False, "simulation already reported success"


def _codex_skill_loop_prompt_context(
    *,
    item: EvalDatasetItem,
    agent_name: AgentName,
    codex_session_id: str,
    trigger_reason: str,
    verification_result: WorkspaceVerificationResult | None,
    simulation_result: SimulationResult | None,
) -> str:
    verification_lines = [
        f"- Verification status: {verification_result.success if verification_result else False}",
    ]
    if verification_result is not None:
        verification_lines.extend(
            [
                f"- Verification name: {verification_result.verification_name}",
                *(
                    [f"- Verification errors: {'; '.join(verification_result.errors)}"]
                    if verification_result.errors
                    else []
                ),
            ]
        )

    simulation_lines = ["- Simulation result: missing"]
    if simulation_result is not None:
        simulation_lines = [f"- Simulation success: {simulation_result.success}"]
        if simulation_result.summary:
            simulation_lines.append(
                f"- Simulation summary: {simulation_result.summary}"
            )

    return "\n".join(
        [
            "Context:",
            f"- Agent: {agent_name.value}",
            f"- Task ID: {item.id}",
            f"- Codex session ID: {codex_session_id}",
            f"- Trigger: {trigger_reason}",
            *verification_lines,
            *simulation_lines,
            "",
            "Workspace files to inspect:",
            "- journal.md",
            "- prompt.md",
            "- validation_results.json",
            "- simulation_result.json",
            "- suggested_skills/",
        ]
    )


def _build_codex_skill_loop_prompt(
    *,
    stage: str,
    item: EvalDatasetItem,
    agent_name: AgentName,
    codex_session_id: str,
    trigger_reason: str,
    verification_result: WorkspaceVerificationResult | None,
    simulation_result: SimulationResult | None,
) -> str:
    base_prompt = _load_codex_skill_loop_prompt(stage)
    runtime_context = _codex_skill_loop_prompt_context(
        item=item,
        agent_name=agent_name,
        codex_session_id=codex_session_id,
        trigger_reason=trigger_reason,
        verification_result=verification_result,
        simulation_result=simulation_result,
    )
    return f"{base_prompt}\n\n{runtime_context}\n"


def _write_codex_skill_loop_prompt(
    *,
    workspace_dir: Path,
    stage: str,
    prompt_text: str,
) -> Path:
    prompt_dir = workspace_dir / "logs" / "skill_loop"
    prompt_dir.mkdir(parents=True, exist_ok=True)
    prompt_path = prompt_dir / f"{stage}.md"
    prompt_path.write_text(prompt_text, encoding="utf-8")
    return prompt_path


def _codex_skill_loop_events_path(workspace_dir: Path) -> Path:
    return workspace_dir / "logs" / "skill_loop" / "events.jsonl"


def _read_optional_text(path: Path | None) -> str:
    if path is None or not path.exists():
        return ""
    try:
        return path.read_text(encoding="utf-8")
    except Exception:
        return ""


def _record_codex_skill_loop_event(
    *,
    workspace_dir: Path,
    event: SkillSelfReflectionEvent | SkillUpdateEvent,
) -> None:
    _append_jsonl_record(
        _codex_skill_loop_events_path(workspace_dir),
        event.model_dump(mode="json"),
    )


def _codex_skill_loop_capture_root(
    *,
    log_context: RunnerLogContext,
    eval_log_key: str | None,
    workspace_dir: Path,
) -> Path:
    if log_context.session_log_root is not None and eval_log_key:
        return log_context.session_log_root / eval_log_key / "codex"
    return workspace_dir / "logs" / "codex"


def _codex_result_failure_reason(
    result: CodexExecRunResult | None,
    *,
    stage: str,
    exception_text: str | None = None,
) -> str | None:
    if exception_text:
        return f"codex skill loop {stage} failed: {exception_text}"
    if result is None:
        return f"codex skill loop {stage} failed"
    if result.timed_out:
        return f"codex skill loop {stage} timed out"
    if result.return_code not in {0, None}:
        return f"codex exited with code {result.return_code}"
    return None


def _skill_self_reflection_event(
    *,
    codex_session_id: str,
    task_id: str,
    agent_name: AgentName,
    trigger_reason: str,
    prompt_path: Path,
    output_path: Path | None,
    reflection_text: str,
    simulation_success: bool | None,
    verification_success: bool | None,
    primary_session_id: str | None,
) -> SkillSelfReflectionEvent:
    return SkillSelfReflectionEvent(
        episode_id=codex_session_id,
        user_session_id=primary_session_id or codex_session_id,
        agent_id=agent_name.value,
        codex_session_id=codex_session_id,
        task_id=task_id,
        agent_name=agent_name.value,
        trigger_reason=trigger_reason,
        prompt_path=prompt_path.as_posix(),
        output_path=output_path.as_posix() if output_path is not None else None,
        reflection_text=reflection_text,
        simulation_success=simulation_success,
        verification_success=verification_success,
    )


def _skill_update_event(
    *,
    codex_session_id: str,
    task_id: str,
    agent_name: AgentName,
    trigger_reason: str,
    prompt_path: Path,
    output_path: Path | None,
    skill_update_text: str,
    updated_skill_paths: list[str],
    simulation_success: bool | None,
    verification_success: bool | None,
    primary_session_id: str | None,
) -> SkillUpdateEvent:
    return SkillUpdateEvent(
        episode_id=codex_session_id,
        user_session_id=primary_session_id or codex_session_id,
        agent_id=agent_name.value,
        codex_session_id=codex_session_id,
        task_id=task_id,
        agent_name=agent_name.value,
        trigger_reason=trigger_reason,
        prompt_path=prompt_path.as_posix(),
        output_path=output_path.as_posix() if output_path is not None else None,
        skill_update_text=skill_update_text,
        updated_skill_paths=updated_skill_paths,
        simulation_success=simulation_success,
        verification_success=verification_success,
    )


def _codex_resume_output_path(loop_root: Path, stage: str) -> Path:
    return loop_root / f"{stage}_last_message.md"


async def _run_codex_skill_loop(
    *,
    item: EvalDatasetItem,
    agent_name: AgentName,
    workspace_dir: Path,
    codex_runtime_root: Path,
    log_context: RunnerLogContext,
    baseline_snapshot,
    codex_trace_artifacts: CodexSessionTraceArtifact | None,
    primary_session_id: str | None = None,
    launch_return_code: int | None,
    verification_result: WorkspaceVerificationResult | None,
    log,
    deps: dict[str, Any] | None = None,
) -> tuple[CodexSkillLoopSummary, CodexSessionTraceArtifact | None]:
    deps = deps or {}
    append_readable_log_line = deps.get(
        "append_readable_log_line", _append_readable_log_line
    )
    capture_latest_codex_session_artifacts = deps.get(
        "capture_latest_codex_session_artifacts"
    )
    if capture_latest_codex_session_artifacts is None:
        from evals.logic.codex_session_trace import (
            capture_latest_codex_session_artifacts as capture_latest_codex_session_artifacts,
        )
    resume_codex_exec = deps.get("resume_codex_exec", _resume_codex_exec)
    resolve_codex_home_root = deps.get(
        "resolve_codex_home_root", _resolve_codex_home_root
    )
    record_codex_skill_loop_event = deps.get(
        "record_codex_skill_loop_event", _record_codex_skill_loop_event
    )
    eval_log_key = deps.get("eval_log_key")

    summary = CodexSkillLoopSummary(enabled=is_coder_agent(agent_name))
    simulation_result = _load_workspace_simulation_result(workspace_dir)
    summary.simulation_success = (
        simulation_result.success if simulation_result is not None else None
    )

    summary.primary_turn = CodexSkillLoopTurn(
        stage="primary",
        session_id=(
            codex_trace_artifacts.session_id
            if codex_trace_artifacts is not None and codex_trace_artifacts.session_id
            else primary_session_id
        ),
        return_code=launch_return_code,
        timed_out=launch_return_code == 124,
        prompt_path=str(workspace_dir / "prompt.md"),
        simulation_success=summary.simulation_success,
        verification_success=bool(
            verification_result.success if verification_result is not None else False
        ),
        failure_reason=None,
    )

    should_run, trigger_reason = _codex_skill_loop_needed(
        agent_name=agent_name,
        launch_return_code=launch_return_code,
        verification_result=verification_result,
        simulation_result=simulation_result,
    )
    summary.triggered = should_run
    summary.trigger_reason = trigger_reason
    if not should_run:
        return summary, codex_trace_artifacts

    if codex_trace_artifacts is None or not codex_trace_artifacts.session_id:
        summary.failure_reason = "missing Codex session trace"
        append_readable_log_line(
            "CODEX_SKILL_LOOP_FAILED "
            f"stage=primary session_id={_sanitize_readable_text(primary_session_id or '')} "
            f"reason={_sanitize_readable_text(summary.failure_reason)}",
            log_context=log_context,
            eval_log_key=eval_log_key,
        )
        log.warning(
            "codex_skill_loop_missing_session_trace",
            session_id=primary_session_id or "",
            trigger_reason=trigger_reason,
        )
        return summary, codex_trace_artifacts

    codex_session_id = codex_trace_artifacts.session_id
    loop_timeout_seconds = _codex_skill_loop_timeout_seconds(agent_name)
    loop_root = workspace_dir / "logs" / "skill_loop"
    loop_root.mkdir(parents=True, exist_ok=True)
    events_path = _codex_skill_loop_events_path(workspace_dir)
    summary.events_path = events_path.as_posix()

    self_analysis_prompt_text = _build_codex_skill_loop_prompt(
        stage="self_analysis",
        item=item,
        agent_name=agent_name,
        codex_session_id=codex_session_id,
        trigger_reason=trigger_reason,
        verification_result=verification_result,
        simulation_result=simulation_result,
    )
    self_analysis_prompt_path = _write_codex_skill_loop_prompt(
        workspace_dir=workspace_dir,
        stage="self_analysis",
        prompt_text=self_analysis_prompt_text,
    )
    append_readable_log_line(
        "CODEX_SKILL_LOOP_STAGE "
        f"stage=self_analysis session_id={codex_session_id} "
        f"prompt_path={_sanitize_readable_text(str(self_analysis_prompt_path))}",
        log_context=log_context,
        eval_log_key=eval_log_key,
    )
    self_analysis_output_path = _codex_resume_output_path(loop_root, "self_analysis")
    self_analysis_result: CodexExecRunResult | None = None
    self_analysis_exception_text: str | None = None
    try:
        self_analysis_result = await asyncio.to_thread(
            resume_codex_exec,
            workspace_dir,
            self_analysis_prompt_text,
            task_id=item.id,
            codex_session_id=codex_session_id,
            agent_name=agent_name,
            session_id=codex_session_id,
            runtime_root=codex_runtime_root,
            yolo=False,
            timeout_seconds=loop_timeout_seconds,
            output_last_message_path=self_analysis_output_path,
            reasoning_effort="xhigh",
        )
    except Exception as exc:
        self_analysis_exception_text = _sanitize_readable_text(exc)

    self_analysis_failure_reason = _codex_result_failure_reason(
        self_analysis_result,
        stage="self-analysis",
        exception_text=self_analysis_exception_text,
    )
    if self_analysis_exception_text is None and self_analysis_failure_reason is None:
        try:
            codex_trace_artifacts = await asyncio.to_thread(
                capture_latest_codex_session_artifacts,
                workspace_dir=workspace_dir,
                artifact_root=_codex_skill_loop_capture_root(
                    log_context=log_context,
                    eval_log_key=eval_log_key,
                    workspace_dir=workspace_dir,
                ),
                baseline_snapshot=baseline_snapshot,
                launched_after_ns=None,
                sessions_root=resolve_codex_home_root(
                    task_id=item.id,
                    session_id=codex_session_id,
                    runtime_root=codex_runtime_root,
                )
                / ".codex"
                / "sessions",
            )
        except Exception as exc:
            log.warning(
                "codex_skill_loop_trace_capture_failed",
                session_id=codex_session_id,
                stage="self_analysis",
                error=_sanitize_readable_text(exc),
            )
            codex_trace_artifacts = None
        if codex_trace_artifacts is not None and codex_trace_artifacts.session_id:
            codex_session_id = codex_trace_artifacts.session_id

    self_reflection_text = _read_optional_text(
        self_analysis_result.output_last_message_path
        if self_analysis_result is not None
        else self_analysis_output_path
    )
    record_codex_skill_loop_event(
        workspace_dir=workspace_dir,
        event=_skill_self_reflection_event(
            codex_session_id=codex_session_id,
            task_id=item.id,
            agent_name=agent_name,
            trigger_reason=trigger_reason,
            prompt_path=self_analysis_prompt_path,
            output_path=(
                self_analysis_result.output_last_message_path
                if self_analysis_result is not None
                else self_analysis_output_path
            ),
            reflection_text=self_reflection_text,
            simulation_success=summary.simulation_success,
            verification_success=bool(
                verification_result.success
                if verification_result is not None
                else False
            ),
            primary_session_id=primary_session_id,
        ),
    )
    summary.event_count += 1
    summary.self_analysis_turn = CodexSkillLoopTurn(
        stage="self_analysis",
        session_id=codex_session_id,
        return_code=(
            self_analysis_result.return_code
            if self_analysis_result is not None
            else None
        ),
        timed_out=(
            self_analysis_result.timed_out
            if self_analysis_result is not None
            else False
        ),
        prompt_path=self_analysis_prompt_path.as_posix(),
        output_path=(
            self_analysis_result.output_last_message_path.as_posix()
            if self_analysis_result is not None
            and self_analysis_result.output_last_message_path is not None
            else self_analysis_output_path.as_posix()
        ),
        simulation_success=summary.simulation_success,
        verification_success=bool(
            verification_result.success if verification_result is not None else False
        ),
        failure_reason=self_analysis_failure_reason,
    )
    if self_analysis_failure_reason is not None:
        summary.failure_reason = self_analysis_failure_reason
        append_readable_log_line(
            "CODEX_SKILL_LOOP_FAILED "
            f"stage=self_analysis session_id={codex_session_id} "
            f"reason={_sanitize_readable_text(summary.failure_reason)}",
            log_context=log_context,
            eval_log_key=eval_log_key,
        )
        log.warning(
            "codex_skill_loop_self_analysis_failed",
            session_id=codex_session_id,
            trigger_reason=trigger_reason,
            failure_reason=summary.failure_reason,
        )
        return summary, codex_trace_artifacts

    skill_update_prompt_text = _build_codex_skill_loop_prompt(
        stage="skill_update",
        item=item,
        agent_name=agent_name,
        codex_session_id=codex_session_id,
        trigger_reason=trigger_reason,
        verification_result=verification_result,
        simulation_result=simulation_result,
    )
    skill_update_prompt_path = _write_codex_skill_loop_prompt(
        workspace_dir=workspace_dir,
        stage="skill_update",
        prompt_text=skill_update_prompt_text,
    )
    append_readable_log_line(
        "CODEX_SKILL_LOOP_STAGE "
        f"stage=skill_update session_id={codex_session_id} "
        f"prompt_path={_sanitize_readable_text(str(skill_update_prompt_path))}",
        log_context=log_context,
        eval_log_key=eval_log_key,
    )
    skill_update_output_path = _codex_resume_output_path(loop_root, "skill_update")
    skill_update_result: CodexExecRunResult | None = None
    skill_update_exception_text: str | None = None
    try:
        skill_update_result = await asyncio.to_thread(
            resume_codex_exec,
            workspace_dir,
            skill_update_prompt_text,
            task_id=item.id,
            codex_session_id=codex_session_id,
            agent_name=agent_name,
            session_id=codex_session_id,
            runtime_root=codex_runtime_root,
            yolo=False,
            timeout_seconds=loop_timeout_seconds,
            output_last_message_path=skill_update_output_path,
            reasoning_effort="xhigh",
        )
    except Exception as exc:
        skill_update_exception_text = _sanitize_readable_text(exc)

    skill_update_failure_reason = _codex_result_failure_reason(
        skill_update_result,
        stage="skill update",
        exception_text=skill_update_exception_text,
    )
    if skill_update_exception_text is None and skill_update_failure_reason is None:
        try:
            codex_trace_artifacts = await asyncio.to_thread(
                capture_latest_codex_session_artifacts,
                workspace_dir=workspace_dir,
                artifact_root=_codex_skill_loop_capture_root(
                    log_context=log_context,
                    eval_log_key=eval_log_key,
                    workspace_dir=workspace_dir,
                ),
                baseline_snapshot=baseline_snapshot,
                launched_after_ns=None,
                sessions_root=resolve_codex_home_root(
                    task_id=item.id,
                    session_id=codex_session_id,
                    runtime_root=codex_runtime_root,
                )
                / ".codex"
                / "sessions",
            )
        except Exception as exc:
            log.warning(
                "codex_skill_loop_trace_capture_failed",
                session_id=codex_session_id,
                stage="skill_update",
                error=_sanitize_readable_text(exc),
            )
            codex_trace_artifacts = None
        if codex_trace_artifacts is not None and codex_trace_artifacts.session_id:
            codex_session_id = codex_trace_artifacts.session_id

    skill_update_text = _read_optional_text(
        skill_update_result.output_last_message_path
        if skill_update_result is not None
        else skill_update_output_path
    )
    updated_skill_paths: list[str] = []
    if codex_trace_artifacts is not None and codex_trace_artifacts.workspace_diff:
        updated_skill_paths = [
            path
            for path in codex_trace_artifacts.workspace_diff.changed_paths
            if path.startswith("suggested_skills/")
        ]
    record_codex_skill_loop_event(
        workspace_dir=workspace_dir,
        event=_skill_update_event(
            codex_session_id=codex_session_id,
            task_id=item.id,
            agent_name=agent_name,
            trigger_reason=trigger_reason,
            prompt_path=skill_update_prompt_path,
            output_path=(
                skill_update_result.output_last_message_path
                if skill_update_result is not None
                else skill_update_output_path
            ),
            skill_update_text=skill_update_text,
            updated_skill_paths=updated_skill_paths,
            simulation_success=summary.simulation_success,
            verification_success=bool(
                verification_result.success
                if verification_result is not None
                else False
            ),
            primary_session_id=primary_session_id,
        ),
    )
    summary.event_count += 1
    summary.skill_update_turn = CodexSkillLoopTurn(
        stage="skill_update",
        session_id=codex_session_id,
        return_code=skill_update_result.return_code
        if skill_update_result is not None
        else None,
        timed_out=skill_update_result.timed_out
        if skill_update_result is not None
        else False,
        prompt_path=skill_update_prompt_path.as_posix(),
        output_path=(
            skill_update_result.output_last_message_path.as_posix()
            if skill_update_result is not None
            and skill_update_result.output_last_message_path is not None
            else skill_update_output_path.as_posix()
        ),
        simulation_success=summary.simulation_success,
        verification_success=bool(
            verification_result.success if verification_result is not None else False
        ),
        failure_reason=skill_update_failure_reason,
    )
    if skill_update_failure_reason is not None:
        summary.failure_reason = skill_update_failure_reason
        append_readable_log_line(
            "CODEX_SKILL_LOOP_FAILED "
            f"stage=skill_update session_id={codex_session_id} "
            f"reason={_sanitize_readable_text(summary.failure_reason)}",
            log_context=log_context,
            eval_log_key=eval_log_key,
        )
        log.warning(
            "codex_skill_loop_skill_update_failed",
            session_id=codex_session_id,
            trigger_reason=trigger_reason,
            failure_reason=summary.failure_reason,
        )
        return summary, codex_trace_artifacts

    append_readable_log_line(
        "CODEX_SKILL_LOOP_COMPLETE "
        f"session_id={codex_session_id} "
        f"trigger_reason={_sanitize_readable_text(trigger_reason)}",
        log_context=log_context,
        eval_log_key=eval_log_key,
    )
    log.info(
        "codex_skill_loop_completed",
        session_id=codex_session_id,
        trigger_reason=trigger_reason,
        simulation_success=summary.simulation_success,
    )
    return summary, codex_trace_artifacts
