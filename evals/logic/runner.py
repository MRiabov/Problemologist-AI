import argparse
import asyncio
import contextlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Any

import httpx
import yaml
from dotenv import load_dotenv
from pyrate_limiter import Duration, Limiter, Rate

from controller.agent.dspy_utils import (
    evaluate_formula,
    map_events_to_prediction,
)
from controller.agent.node_entry_validation import (
    reviewer_handover_custom_check_from_session_id,
)
from controller.agent.review_handover import validate_reviewer_handover
from controller.agent.reward import (
    AgentRewardConfig,
    MilestoneConfig,
    load_reward_config,
)
from controller.clients.worker import WorkerClient
from evals.logic.models import (
    AgentEvalSpec,
    EvalDatasetItem,
    GitEvalConfig,
    GitStatusExpectation,
    HardCheckAggregate,
)
from evals.logic.review_checks import (
    planner_artifacts_present as _planner_artifacts_present,
)
from evals.logic.review_checks import (
    requires_expected_review_decision as _requires_expected_review_decision,
)
from evals.logic.review_checks import (
    review_artifact_pending as _review_artifact_pending,
)
from evals.logic.review_checks import (
    review_artifacts_complete_for_prefix as _review_artifacts_complete_for_prefix,
)
from evals.logic.review_checks import (
    review_expectation_error as _review_expectation_error,
)
from evals.logic.specs import (
    AGENT_SPECS,
    JUDGE_REVIEWER_CHAIN,
    PLANNER_REQUIRED_FILES,
)
from evals.logic.workspace import (
    preflight_seeded_entry_contract as _preflight_seeded_entry_contract,
)
from evals.logic.workspace import (
    seed_eval_workspace as _seed_eval_workspace,
)
from shared.enums import (
    AgentName,
    EpisodeStatus,
    EvalMode,
    GenerationKind,
    ReviewDecision,
    SeedMatchMethod,
)
from shared.logging import configure_logging, get_logger
from shared.models.schemas import EpisodeMetadata
from shared.utils.evaluation import analyze_electronics_metrics

ROOT = Path(__file__).resolve().parents[2]

load_dotenv()
# Logging will be configured in main() to support redirection to logs/evals
logger = get_logger(__name__)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
READABLE_AGENT_LOG_FILE: Path | None = None
_READABLE_AGENT_LOG_LOCK = Lock()
SESSION_LOG_ROOT: Path | None = None

_EVAL_KEY_RE = re.compile(r"([a-z]{1,12}-\d{3})", re.IGNORECASE)


def _truncate_text(value: str, *, limit: int = 160) -> str:
    text = value.strip()
    if len(text) <= limit:
        return text
    return text[: limit - 3].rstrip() + "..."


def _sanitize_readable_text(value: Any) -> str:
    if value is None:
        return ""

    text = str(value).replace("\r\n", "\n").replace("\r", "\n").strip()
    if not text:
        return ""

    if len(text) >= 2 and text[0] == text[-1] and text[0] in {'"', "'"}:
        with contextlib.suppress(json.JSONDecodeError):
            decoded = json.loads(text)
            if isinstance(decoded, str):
                text = decoded

    text = text.encode("utf-8", "backslashreplace").decode("unicode_escape")
    text = "".join(ch if ch.isprintable() or ch in "\n\t" else " " for ch in text)
    text = " ".join(part for part in text.replace("\t", " ").split())
    return text.strip()


def _short_run_label(episode: dict[str, Any]) -> str:
    metadata = episode.get("metadata_vars") or {}
    if not isinstance(metadata, dict):
        metadata = {}

    for candidate in (
        metadata.get("benchmark_id"),
        metadata.get("worker_session_id"),
        episode.get("id"),
    ):
        if isinstance(candidate, str) and candidate.strip():
            return candidate.strip()[:7]

    return "unknown"


def _parse_trace_json_content(raw_content: Any) -> dict[str, Any] | None:
    if not isinstance(raw_content, str):
        return None

    with contextlib.suppress(json.JSONDecodeError):
        parsed = json.loads(raw_content)
        if isinstance(parsed, dict):
            return parsed

    return None


def _parse_task_id_filters(raw_task_id_filters: list[str] | None) -> set[str]:
    """Parse task ID filters from repeated --task-id arguments.

    Supported forms per argument:
    - single ID: "id-1"
    - comma-separated: "id-1,id-2"
    - bracket list: "[id-1,id-2]" or '["id-1","id-2"]'
    """
    if not raw_task_id_filters:
        return set()

    selected_ids: set[str] = set()
    for raw_filter in raw_task_id_filters:
        text = raw_filter.strip()
        if not text:
            continue

        values_to_parse = text
        if text.startswith("[") and text.endswith("]"):
            with contextlib.suppress(json.JSONDecodeError):
                parsed = json.loads(text)
                if isinstance(parsed, list):
                    for item in parsed:
                        item_text = str(item).strip()
                        if item_text:
                            selected_ids.add(item_text.strip("'\""))
                    continue
            values_to_parse = text[1:-1]

        for candidate in values_to_parse.split(","):
            normalized = candidate.strip().strip("'\"")
            if normalized:
                selected_ids.add(normalized)

    return selected_ids


def _format_tool_args(trace: dict[str, Any]) -> str:
    parsed = _parse_trace_json_content(trace.get("content"))
    if parsed is None:
        raw = _sanitize_readable_text(trace.get("content"))
        return _truncate_text(raw, limit=120) if raw else ""

    kwargs = parsed.get("kwargs")
    args = parsed.get("args")
    parts: list[str] = []

    if isinstance(kwargs, dict):
        preferred_keys = (
            "path",
            "file_path",
            "target_path",
            "directory",
            "command",
            "cmd",
            "query",
            "prompt",
            "plan_path",
            "script_path",
            "backend",
        )
        for key in preferred_keys:
            value = kwargs.get(key)
            if value in (None, "", [], {}, ()):
                continue
            parts.append(
                f"{key}={_truncate_text(_sanitize_readable_text(value), limit=80)}"
            )

        if not parts:
            for key, value in kwargs.items():
                if value in (None, "", [], {}, ()):
                    continue
                if key in {"content", "review_content"}:
                    continue
                parts.append(
                    f"{key}={_truncate_text(_sanitize_readable_text(value), limit=60)}"
                )
                if len(parts) >= 3:
                    break

    if not parts and isinstance(args, list):
        for value in args[:3]:
            parts.append(_truncate_text(_sanitize_readable_text(value), limit=60))

    return " ".join(parts)


def _format_readable_trace_line(
    *,
    episode: dict[str, Any],
    trace: dict[str, Any],
    default_agent_name: str,
    detail_mode: str = "default",
) -> str | None:
    trace_type = str(trace.get("trace_type") or "").upper()
    run_label = _short_run_label(episode)
    metadata = trace.get("metadata_vars") or {}
    if not isinstance(metadata, dict):
        metadata = {}

    if trace_type == "LLM_END" and detail_mode == "default":
        agent_name = _sanitize_readable_text(trace.get("name")) or default_agent_name
        prefix = f"{agent_name} | {run_label} | "
        text = _sanitize_readable_text(trace.get("content"))
        if not text:
            return None
        return prefix + text

    if trace_type != "TOOL_START":
        return None

    prefix = f"{default_agent_name} | {run_label} | "
    error_text = _sanitize_readable_text(metadata.get("error"))
    if detail_mode == "error":
        if not error_text:
            return None
        return (
            prefix
            + "TOOL "
            + _sanitize_readable_text(trace.get("name"))
            + (f" ERROR {error_text}")
        )

    observation_text = _sanitize_readable_text(metadata.get("observation"))
    if detail_mode == "result":
        if not observation_text:
            return None
        return (
            prefix
            + "RESULT "
            + _sanitize_readable_text(trace.get("name"))
            + " "
            + _truncate_text(observation_text, limit=220)
        )

    tool_name = _sanitize_readable_text(trace.get("name"))
    tool_args = _format_tool_args(trace)
    base = prefix + "TOOL " + tool_name
    if tool_args:
        base += " " + tool_args
    return base


def _resolve_eval_log_key(*, task_id: str, session_id: str = "") -> str:
    for source in (task_id, session_id):
        if not source:
            continue
        match = _EVAL_KEY_RE.search(source)
        if match:
            return match.group(1).lower()
    fallback = re.sub(r"[^A-Za-z0-9._-]+", "_", task_id.strip())
    return fallback[:80] or "unscoped"


def _append_readable_log_line(line: str, *, eval_log_key: str | None = None) -> None:
    if READABLE_AGENT_LOG_FILE is None and SESSION_LOG_ROOT is None:
        return

    with _READABLE_AGENT_LOG_LOCK:
        if READABLE_AGENT_LOG_FILE is not None:
            READABLE_AGENT_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
            with READABLE_AGENT_LOG_FILE.open("a", encoding="utf-8") as handle:
                handle.write(line.rstrip() + "\n")
        if SESSION_LOG_ROOT is not None and eval_log_key:
            session_file = SESSION_LOG_ROOT / eval_log_key / "readable_agent_logs.log"
            session_file.parent.mkdir(parents=True, exist_ok=True)
            with session_file.open("a", encoding="utf-8") as handle:
                handle.write(line.rstrip() + "\n")


def _write_eval_session_metadata(
    *,
    eval_log_key: str | None,
    payload: dict[str, Any],
) -> None:
    if SESSION_LOG_ROOT is None or not eval_log_key:
        return
    metadata_path = SESSION_LOG_ROOT / eval_log_key / "session_metadata.json"
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(payload, indent=2, sort_keys=True),
        encoding="utf-8",
    )


async def _handle_electronics_metrics(
    worker: WorkerClient, session_id: str, agent_stats: dict
):
    """Evaluates electronics metrics and updates stats."""
    v, i, e = await analyze_electronics_metrics(worker, session_id)
    agent_stats["electrical_validity_rate"] += v
    agent_stats["wire_integrity_rate"] += i
    agent_stats["power_efficiency_score"] += e


# Mapping of agent names to their specific metric handlers
METRIC_HANDLERS = {}


def _load_agent_reward_configs() -> dict[AgentName, AgentRewardConfig]:
    try:
        reward_cfg = load_reward_config()
    except Exception:
        logger.exception("reward_config_load_failed")
        return {}

    configs: dict[AgentName, AgentRewardConfig] = {}
    for group in (reward_cfg.benchmark, reward_cfg.engineer, reward_cfg.shared):
        for name, cfg in group.items():
            with contextlib.suppress(ValueError):
                configs[AgentName(name)] = cfg
    return configs


def _extract_episode_events(episode: dict[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(episode, dict):
        return []

    traces = episode.get("traces")
    if not isinstance(traces, list):
        return []

    events: list[dict[str, Any]] = []
    for trace in traces:
        if not isinstance(trace, dict):
            continue
        if str(trace.get("trace_type") or "").upper() != "EVENT":
            continue
        metadata = trace.get("metadata_vars")
        if isinstance(metadata, dict):
            events.append(metadata)
            continue
        parsed = _parse_trace_json_content(trace.get("content"))
        if isinstance(parsed, dict):
            events.append(parsed)
    return events


def _extract_episode_cost_usd(episode: dict[str, Any] | None) -> float | None:
    """Return accumulated USD spend from episode metadata when available."""
    if not isinstance(episode, dict):
        return None

    metadata_raw = episode.get("metadata_vars")
    if not isinstance(metadata_raw, dict):
        return None

    with contextlib.suppress(Exception):
        metadata = EpisodeMetadata.model_validate(metadata_raw)
        additional = dict(metadata.additional_info or {})
        credit_usage = additional.get("credit_usage")
        if isinstance(credit_usage, dict):
            raw_cost = credit_usage.get("total_cost_usd")
            if isinstance(raw_cost, int | float):
                return float(raw_cost)

    return None


def _resolve_check_value(
    check_name: str,
    metrics: dict[str, Any],
    *,
    success: bool,
) -> Any:
    aliases: dict[str, tuple[str, ...]] = {
        "script_compiles": ("script_compiled", "compiled"),
        "simulation_result": ("simulation_success", "simulation_ran"),
        "cad_geometry_valid": ("geometry_valid", "cad_geometry_valid"),
        "reviewer_accepted": ("accepted", "reviewer_accepted"),
        "engineer_implemented_successfully": (
            "engineer_implemented_successfully",
            "simulation_success",
        ),
    }
    for key in (check_name, *aliases.get(check_name, ())):
        if key in metrics:
            return metrics[key]
    if check_name == "task_success":
        return success
    return None


def _normalize_checklist_value(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, int | float):
        return max(0.0, min(1.0, float(value)))
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized == "pass":
            return 1.0
        if normalized == "fail":
            return 0.0
        if normalized in {"not_applicable", "n/a", "na"}:
            return 1.0
    return None


def _resolve_metric_value_for_check(
    check_name: str,
    metrics: dict[str, Any],
    *,
    success: bool,
) -> Any:
    if check_name.startswith("checklist."):
        checklist_key = check_name.split(".", 1)[1]
        checklist = metrics.get("checklist")
        if isinstance(checklist, dict):
            return _normalize_checklist_value(checklist.get(checklist_key))
        return None
    return _resolve_check_value(check_name, metrics, success=success)


def _check_needs_constraints(check_cfg: MilestoneConfig) -> bool:
    formulas = [check_cfg.penalty_formula, check_cfg.formula]
    return any(
        isinstance(formula, str)
        and ("max_unit_cost" in formula or "max_weight" in formula)
        for formula in formulas
    )


def _score_milestone_check(
    *,
    check_name: str,
    check_cfg: MilestoneConfig,
    metrics: dict[str, Any],
    success: bool,
) -> tuple[float, bool]:
    raw_value = _resolve_metric_value_for_check(check_name, metrics, success=success)
    if check_cfg.partial:
        formula = check_cfg.penalty_formula or check_cfg.formula
        if formula:
            score = evaluate_formula(formula, metrics)
        elif isinstance(raw_value, int | float) and not isinstance(raw_value, bool):
            score = float(raw_value)
        else:
            score = 1.0 if bool(raw_value) else 0.0
    else:
        if isinstance(raw_value, int | float) and not isinstance(raw_value, bool):
            score = float(raw_value)
        else:
            score = 1.0 if bool(raw_value) else 0.0

    score = max(0.0, min(1.0, float(score)))
    return score, score >= 0.999


def _extract_events_from_episodes(
    episodes: list[dict[str, Any] | None],
) -> list[dict[str, Any]]:
    merged: list[dict[str, Any]] = []
    for episode in episodes:
        merged.extend(_extract_episode_events(episode))
    return merged


async def _load_constraint_context(session_id: str) -> dict[str, float]:
    if not session_id:
        return {}

    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        content = await worker.read_file("benchmark_definition.yaml")
    finally:
        await worker.aclose()

    if not isinstance(content, str) or content.startswith("Error:"):
        return {}

    with contextlib.suppress(Exception):
        parsed = yaml.safe_load(content)
        if not isinstance(parsed, dict):
            return {}
        constraints = parsed.get("constraints")
        if not isinstance(constraints, dict):
            return {}
        context: dict[str, float] = {}
        max_unit_cost = constraints.get("max_unit_cost")
        max_weight_g = constraints.get("max_weight_g", constraints.get("max_weight"))
        if isinstance(max_unit_cost, int | float):
            context["max_unit_cost"] = float(max_unit_cost)
        if isinstance(max_weight_g, int | float):
            context["max_weight_g"] = float(max_weight_g)
            context["max_weight"] = float(max_weight_g)
        return context

    return {}


async def _collect_metrics_for_checks(
    *,
    check_configs: dict[str, MilestoneConfig],
    success: bool,
    session_id: str,
    episode_data: dict[str, Any] | None,
    extra_episodes: list[dict[str, Any] | None] | None = None,
    agent_name: AgentName | None = None,
) -> dict[str, Any]:
    episodes = [episode_data]
    if extra_episodes:
        episodes.extend(extra_episodes)

    events = _extract_events_from_episodes(episodes)
    metrics_model = map_events_to_prediction(events)
    metrics = metrics_model.model_dump(mode="json")
    metrics["task_success"] = success

    check_names = set(check_configs.keys())
    needs_worker_checks = check_names.intersection(
        {"review_artifacts_complete", "plan_artifacts_present"}
    )
    if needs_worker_checks and session_id:
        worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
        try:
            if "review_artifacts_complete" in needs_worker_checks and agent_name:
                review_prefix = AGENT_SPECS.get(agent_name).review_filename_prefix
                if review_prefix:
                    metrics[
                        "review_artifacts_complete"
                    ] = await _review_artifacts_complete_for_prefix(
                        worker=worker,
                        review_filename_prefix=review_prefix,
                    )

            if "plan_artifacts_present" in needs_worker_checks and agent_name:
                required_files = PLANNER_REQUIRED_FILES.get(agent_name, ())
                metrics["plan_artifacts_present"] = await _planner_artifacts_present(
                    worker=worker,
                    required_files=required_files,
                )
        finally:
            await worker.aclose()

    if any(_check_needs_constraints(check) for check in check_configs.values()):
        metrics.update(await _load_constraint_context(session_id))

    return metrics


async def _record_hard_check_outcomes(
    *,
    stats: dict[AgentName, Any],
    reward_agent_configs: dict[AgentName, AgentRewardConfig],
    agent_name: AgentName,
    item: EvalDatasetItem,
    success: bool,
    episode_data: dict[str, Any] | None,
    session_id: str,
) -> bool | None:
    cfg = reward_agent_configs.get(agent_name)
    if cfg is None or not cfg.hard_checks:
        return None

    metrics = await _collect_metrics_for_checks(
        check_configs=cfg.hard_checks,
        success=success,
        session_id=session_id,
        episode_data=episode_data,
        agent_name=agent_name,
    )

    hard_check_stats = stats[agent_name].setdefault("hard_checks", {})
    seed_id = item.id
    all_passed = True
    for check_name, check_cfg in cfg.hard_checks.items():
        bucket = hard_check_stats.setdefault(
            check_name,
            HardCheckAggregate().model_dump(mode="json"),
        )
        bucket["total"] += 1

        _, passed = _score_milestone_check(
            check_name=check_name,
            check_cfg=check_cfg,
            metrics=metrics,
            success=success,
        )
        if passed:
            bucket["passed"] += 1
        elif seed_id not in bucket["failed_seeds"]:
            bucket["failed_seeds"].append(seed_id)
            all_passed = False

    return all_passed


async def _record_judge_outcomes(
    *,
    stats: dict[AgentName, Any],
    reward_agent_configs: dict[AgentName, AgentRewardConfig],
    agent_name: AgentName,
    item: EvalDatasetItem,
    success: bool,
    session_id: str,
    episode_data: dict[str, Any] | None,
    extra_episodes: list[dict[str, Any] | None] | None = None,
) -> None:
    cfg = reward_agent_configs.get(agent_name)
    if cfg is None:
        return

    judge_checks = cfg.judge_evaluation.all_milestones()
    if not judge_checks:
        return

    metrics = await _collect_metrics_for_checks(
        check_configs=judge_checks,
        success=success,
        session_id=session_id,
        episode_data=episode_data,
        extra_episodes=extra_episodes,
        agent_name=agent_name,
    )

    judge_stats = stats[agent_name].setdefault("judge_checks", {})
    seed_id = item.id
    for check_name, check_cfg in judge_checks.items():
        bucket = judge_stats.setdefault(
            check_name,
            HardCheckAggregate().model_dump(mode="json"),
        )
        bucket["total"] += 1
        _, passed = _score_milestone_check(
            check_name=check_name,
            check_cfg=check_cfg,
            metrics=metrics,
            success=success,
        )
        if passed:
            bucket["passed"] += 1
        elif seed_id not in bucket["failed_seeds"]:
            bucket["failed_seeds"].append(seed_id)


def _episode_terminal(status: str | None) -> bool:
    if not status:
        return False
    return status in {
        EpisodeStatus.COMPLETED,
        EpisodeStatus.FAILED,
        EpisodeStatus.CANCELLED,
    }


def _planned_counts_as_success(agent_name: AgentName, spec: AgentEvalSpec) -> bool:
    if agent_name == AgentName.BENCHMARK_PLAN_REVIEWER:
        return True
    if spec.mode == EvalMode.BENCHMARK and not _requires_expected_review_decision(spec):
        return True
    return agent_name in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.BENCHMARK_PLANNER,
    }


async def _wait_for_controller_ready(
    timeout_seconds: float = 60.0, poll_interval_seconds: float = 1.0
) -> None:
    deadline = time.monotonic() + timeout_seconds
    attempt = 0
    last_error: str | None = None

    async with httpx.AsyncClient(timeout=5.0) as client:
        while time.monotonic() < deadline:
            attempt += 1
            try:
                health_resp = await client.get(f"{CONTROLLER_URL}/health")
                health_payload = health_resp.json()
                if (
                    health_resp.status_code == 200
                    and health_payload.get("status") == "healthy"
                ):
                    logger.info("controller_health_check_ok", attempts=attempt)
                    return
                last_error = (
                    f"status_code={health_resp.status_code}, "
                    f"status={health_payload.get('status')}"
                )
            except Exception as exc:
                last_error = str(exc)

            logger.warning(
                "controller_health_check_retry", attempt=attempt, error=last_error
            )
            await asyncio.sleep(poll_interval_seconds)

    raise RuntimeError(
        "Controller did not become healthy within "
        f"{timeout_seconds}s. last_error={last_error}"
    )


async def _wait_for_worker_ready(
    timeout_seconds: float = 60.0, poll_interval_seconds: float = 1.0
) -> None:
    deadline = time.monotonic() + timeout_seconds
    attempt = 0
    last_error: str | None = None
    temp_client = WorkerClient(base_url=WORKER_LIGHT_URL, session_id="healthcheck")

    while time.monotonic() < deadline:
        attempt += 1
        try:
            health = await temp_client.get_health()
            if health.get("status") == "healthy":
                logger.info("worker_health_check_ok", attempts=attempt)
                return
            last_error = f"status={health.get('status')}"
        except Exception as exc:
            last_error = str(exc)

        logger.warning("worker_health_check_retry", attempt=attempt, error=last_error)
        await asyncio.sleep(poll_interval_seconds)

    raise RuntimeError(
        "Worker did not become healthy within "
        f"{timeout_seconds}s. last_error={last_error}"
    )


async def _fetch_episode(client: httpx.AsyncClient, episode_id: str) -> dict[str, Any]:
    response = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
    response.raise_for_status()
    return response.json()


async def _request_episode_interrupt(
    episode_id: str,
    *,
    log,
) -> None:
    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            resp = await client.post(
                f"{CONTROLLER_URL}/episodes/{episode_id}/interrupt"
            )
        if resp.status_code in {200, 202}:
            log.warning("eval_interrupt_requested", episode_id=episode_id)
            return
        log.warning(
            "eval_interrupt_request_failed",
            episode_id=episode_id,
            status_code=resp.status_code,
            response_text=resp.text,
        )
    except Exception:
        log.exception("eval_interrupt_request_exception", episode_id=episode_id)


def _trace_names_lower(episode: dict[str, Any]) -> set[str]:
    traces = episode.get("traces") or []
    names = set()
    for trace in traces:
        name = trace.get("name")
        if isinstance(name, str) and name:
            names.add(name.lower())
    return names


def _missing_required_traces(
    required: tuple[AgentName, ...], episode: dict[str, Any]
) -> list[str]:
    names = _trace_names_lower(episode)
    return [trace.value for trace in required if trace.value.lower() not in names]


async def _run_reviewer_episode_for_judge(
    *,
    client: httpx.AsyncClient,
    reviewer_agent: AgentName,
    session_id: str,
    task_description: str,
    lineage: EpisodeMetadata,
    log,
) -> dict[str, Any] | None:
    reviewer_spec = AGENT_SPECS.get(reviewer_agent)
    if reviewer_spec is None:
        log.warning("judge_reviewer_spec_missing", reviewer_agent=reviewer_agent.value)
        return None

    payload = {
        "task": task_description,
        "agent_name": reviewer_agent,
        "start_node": reviewer_agent,
        "session_id": session_id,
        "metadata_vars": lineage.model_dump(exclude_none=True),
    }
    response = await client.post(f"{CONTROLLER_URL}/agent/run", json=payload)
    if response.status_code >= 400:
        log.warning(
            "judge_reviewer_trigger_failed",
            reviewer_agent=reviewer_agent.value,
            status_code=response.status_code,
            response_text=response.text,
            session_id=session_id,
        )
        return None

    response_data = response.json()
    episode_id = str(response_data.get("episode_id") or "")
    if not episode_id:
        log.warning(
            "judge_reviewer_missing_episode_id",
            reviewer_agent=reviewer_agent.value,
            session_id=session_id,
        )
        return None

    log.info(
        "judge_reviewer_triggered",
        reviewer_agent=reviewer_agent.value,
        episode_id=episode_id,
        session_id=session_id,
    )

    max_attempts = 60
    attempt = 0
    latest_episode: dict[str, Any] | None = None
    while attempt < max_attempts:
        attempt += 1
        await asyncio.sleep(5)
        try:
            latest_episode = await _fetch_episode(client, episode_id)
        except Exception:
            log.exception(
                "judge_reviewer_status_fetch_failed",
                reviewer_agent=reviewer_agent.value,
                episode_id=episode_id,
            )
            continue

        status = latest_episode.get("status")
        if status == EpisodeStatus.PLANNED or _episode_terminal(status):
            break

    if latest_episode is None:
        log.warning(
            "judge_reviewer_no_episode_data",
            reviewer_agent=reviewer_agent.value,
            episode_id=episode_id,
        )
        return None

    if attempt >= max_attempts and not _episode_terminal(latest_episode.get("status")):
        log.warning(
            "judge_reviewer_timeout",
            reviewer_agent=reviewer_agent.value,
            episode_id=episode_id,
            max_attempts=max_attempts,
        )

    missing_traces = _missing_required_traces(
        reviewer_spec.required_trace_names, latest_episode
    )
    if missing_traces:
        log.warning(
            "judge_reviewer_missing_traces",
            reviewer_agent=reviewer_agent.value,
            episode_id=episode_id,
            missing_traces=missing_traces,
        )

    return latest_episode


async def _run_reviewer_chain_for_judge(
    *,
    client: httpx.AsyncClient,
    agent_name: AgentName,
    session_id: str,
    task_description: str,
    lineage: EpisodeMetadata,
    log,
) -> list[dict[str, Any]]:
    reviewer_chain = JUDGE_REVIEWER_CHAIN.get(agent_name, ())
    if not reviewer_chain:
        return []

    episodes: list[dict[str, Any]] = []
    for reviewer_agent in reviewer_chain:
        episode = await _run_reviewer_episode_for_judge(
            client=client,
            reviewer_agent=reviewer_agent,
            session_id=session_id,
            task_description=task_description,
            lineage=lineage,
            log=log,
        )
        if episode is not None:
            episodes.append(episode)
    return episodes


async def _completion_contract_error(
    *,
    spec: AgentEvalSpec,
    session_id: str,
) -> str | None:
    manifest_path = spec.required_reviewer_handover_manifest
    expected_stage = spec.required_reviewer_stage
    if not manifest_path or not expected_stage:
        return None

    if spec.materialize_reviewer_handover:
        errors = await reviewer_handover_custom_check_from_session_id(
            session_id=session_id,
            reviewer_label="Execution",
            manifest_path=manifest_path,
            expected_stage=expected_stage,
        )
        if errors:
            return "; ".join(error.message for error in errors)
        return None

    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        return await validate_reviewer_handover(
            worker,
            manifest_path=manifest_path,
            expected_stage=expected_stage,
        )
    finally:
        await worker.aclose()


def _validate_git_status(
    status: Any, expectation: GitStatusExpectation | None
) -> str | None:
    if getattr(status, "error", None):
        return f"git status returned error: {status.error}"

    if expectation is None:
        return None

    if expectation.branch is not None and status.branch != expectation.branch:
        return f"expected branch {expectation.branch!r}, got {status.branch!r}"

    if expectation.is_dirty is not None and status.is_dirty != expectation.is_dirty:
        return f"expected is_dirty={expectation.is_dirty}, got {status.is_dirty}"

    if (
        expectation.is_merging is not None
        and status.is_merging != expectation.is_merging
    ):
        return f"expected is_merging={expectation.is_merging}, got {status.is_merging}"

    if expectation.conflicts is not None:
        actual_conflicts = sorted(status.conflicts)
        expected_conflicts = sorted(expectation.conflicts)
        if actual_conflicts != expected_conflicts:
            return f"expected conflicts={expected_conflicts}, got {actual_conflicts}"

    return None


def _validate_git_commit_result(
    *,
    commit: Any,
    expect_commit_hash: bool,
    action_label: str,
) -> str | None:
    if not commit.success:
        return f"{action_label} returned success=false: {commit.message}"

    has_hash = commit.commit_hash is not None
    if expect_commit_hash and not has_hash:
        return f"{action_label} completed without a commit hash"
    if not expect_commit_hash and has_hash:
        return f"{action_label} unexpectedly produced commit hash {commit.commit_hash}"
    return None


def _apply_default_non_frontend_integration_marker(command: str) -> str:
    """Exclude frontend integration tests unless the command opts in explicitly."""
    try:
        tokens = shlex.split(command, posix=True)
    except ValueError:
        return command

    if not tokens:
        return command

    cmd_index = 0
    while cmd_index < len(tokens) and re.fullmatch(
        r"[A-Za-z_][A-Za-z0-9_]*=.*", tokens[cmd_index]
    ):
        cmd_index += 1

    if cmd_index >= len(tokens):
        return command

    script_token = tokens[cmd_index]
    if script_token not in {
        "./scripts/run_integration_tests.sh",
        "scripts/run_integration_tests.sh",
    }:
        return command

    args = tokens[cmd_index + 1 :]
    if any(arg == "-m" or arg.startswith("-m=") for arg in args):
        return command
    if any(
        "tests/integration/frontend" in arg
        or "tests/e2e" in arg
        or "integration_frontend" in arg
        for arg in args
    ):
        return command

    return command + ' -m "integration_p0 or integration_p1 or integration_p2"'


async def _run_git_eval(
    item: EvalDatasetItem,
    stats: dict[AgentName, Any],
    agent_name: AgentName,
    reward_agent_configs: dict[AgentName, AgentRewardConfig],
):
    task_id = item.id
    log = logger.bind(task_id=task_id, agent_name=agent_name, eval_mode=EvalMode.GIT)
    log.info("eval_start")

    git_eval = item.git_eval or GitEvalConfig()
    session_id = f"eval-git-{task_id}-{uuid.uuid4().hex[:8]}"
    eval_log_key = _resolve_eval_log_key(task_id=task_id, session_id=session_id)
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)

    success = False
    failure_reason = ""

    try:
        await worker.git_init()
        if item.seed_artifact_dir is not None or item.seed_files:
            await _seed_eval_workspace(
                item=item,
                session_id=session_id,
                agent_name=agent_name,
                root=ROOT,
                worker_light_url=WORKER_LIGHT_URL,
                logger=logger,
            )

        if (
            not git_eval.setup_commands
            and item.seed_artifact_dir is None
            and not item.seed_files
        ):
            await worker.write_file(
                "git_eval_note.md",
                f"# Git Eval {task_id}\n\n{item.task.strip()}\n",
                overwrite=True,
            )

        for command in git_eval.setup_commands:
            command_to_run = _apply_default_non_frontend_integration_marker(command)
            if command_to_run != command:
                log.info(
                    "setup_command_adjusted",
                    original_command=command,
                    adjusted_command=command_to_run,
                )

            result = await worker.execute_command(command_to_run, timeout=60)
            if result.timed_out:
                failure_reason = f"setup command timed out: {command_to_run}"
                break
            if result.exit_code != 0:
                stderr = _truncate_text(result.stderr or result.stdout or "")
                failure_reason = (
                    f"setup command failed with exit_code={result.exit_code}: "
                    f"{command_to_run} ({stderr})"
                )
                break

        requires_merge_flow = bool(
            git_eval.resolve_conflicts
            or git_eval.abort_merge
            or git_eval.merge_complete_message is not None
        )
        if not failure_reason and requires_merge_flow:
            merge_status = await worker.git_status()
            failure_reason = _validate_git_status(merge_status, None)
            if not failure_reason:
                if not merge_status.is_merging:
                    failure_reason = "expected setup to leave repository in merge state"
                elif not merge_status.conflicts:
                    failure_reason = (
                        "expected setup to create conflicted files before merge action"
                    )

        if not failure_reason:
            for step in git_eval.resolve_conflicts:
                resolved = await worker.git_resolve(
                    file_path=step.file_path,
                    strategy=step.strategy,
                )
                if not resolved:
                    failure_reason = (
                        f"git resolve failed for {step.file_path} with strategy "
                        f"{step.strategy}"
                    )
                    break

        if not failure_reason and git_eval.abort_merge:
            aborted = await worker.git_merge_abort()
            if not aborted:
                failure_reason = "git merge abort returned success=false"

        commit = None
        if not failure_reason and not git_eval.abort_merge:
            if git_eval.merge_complete_message is not None:
                commit = await worker.git_merge_complete(
                    message=git_eval.merge_complete_message
                )
                action_label = "git merge complete"
            else:
                commit = await worker.git_commit(
                    message=git_eval.commit_message
                    or f"eval({task_id}): git agent baseline"
                )
                action_label = "git commit"

            expected_hash = git_eval.expect_commit_hash
            if expected_hash is None:
                expected_hash = True
            failure_reason = _validate_git_commit_result(
                commit=commit,
                expect_commit_hash=expected_hash,
                action_label=action_label,
            )

        if not failure_reason:
            status = await worker.git_status()
            failure_reason = _validate_git_status(status, git_eval.expected_status)

        if not failure_reason:
            success = True
            log.info("eval_completed", session_id=session_id)
    except Exception as exc:
        failure_reason = str(exc)
    finally:
        await worker.aclose()

    if not success:
        log.error("eval_failed", reason=failure_reason, session_id=session_id)
    _write_eval_session_metadata(
        eval_log_key=eval_log_key,
        payload={
            "agent_name": agent_name.value,
            "episode_id": None,
            "eval_mode": EvalMode.GIT.value,
            "failure_reason": failure_reason,
            "session_id": session_id,
            "success": success,
            "task_id": task_id,
        },
    )

    agent_stats = stats[agent_name]
    agent_stats["total"] += 1
    if success:
        agent_stats["success"] += 1
    await _record_hard_check_outcomes(
        stats=stats,
        reward_agent_configs=reward_agent_configs,
        agent_name=agent_name,
        item=item,
        success=success,
        episode_data=None,
        session_id=session_id,
    )


async def run_single_eval(
    item: EvalDatasetItem,
    agent_name: AgentName,
    stats: dict[AgentName, Any],
    reward_agent_configs: dict[AgentName, AgentRewardConfig],
    verbose: bool = False,
    run_judge: bool = False,
    run_reviewers_with_judge: bool = False,
):
    """
    Runs a single evaluation task for a specific agent type.
    """
    spec = AGENT_SPECS[agent_name]

    if spec.mode == EvalMode.GIT:
        await _run_git_eval(
            item,
            stats,
            agent_name,
            reward_agent_configs,
        )
        return

    task_id = item.id
    task_description = item.task
    lineage = EpisodeMetadata(
        seed_id=task_id,
        seed_dataset=str(item.seed_dataset) if item.seed_dataset else None,
        seed_match_method=SeedMatchMethod.RUNTIME_EXPLICIT,
        generation_kind=GenerationKind.SEEDED_EVAL,
        parent_seed_id=task_id,
        disable_sidecars=True,
    )

    log = logger.bind(
        task_id=task_id,
        agent_name=agent_name,
        eval_mode=spec.mode,
    )
    log.info("eval_start")
    readable_agent = spec.start_node or spec.request_agent_name or agent_name
    readable_agent_name = (
        readable_agent.value
        if isinstance(readable_agent, AgentName)
        else str(readable_agent).strip()
    )

    success = False
    session_id = ""
    episode_id = ""
    last_episode_data: dict[str, Any] | None = None
    eval_log_key = _resolve_eval_log_key(task_id=task_id)

    async with httpx.AsyncClient(timeout=60.0) as client:
        try:
            if spec.mode == EvalMode.BENCHMARK:
                session_id = f"eval-{task_id}-{uuid.uuid4().hex[:8]}"
                eval_log_key = _resolve_eval_log_key(
                    task_id=task_id, session_id=session_id
                )
                await _seed_eval_workspace(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    root=ROOT,
                    worker_light_url=WORKER_LIGHT_URL,
                    logger=logger,
                )
                await _preflight_seeded_entry_contract(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    spec=spec,
                    worker_light_url=WORKER_LIGHT_URL,
                    logger=logger,
                )
                url = f"{CONTROLLER_URL}/benchmark/generate"
                payload = {
                    "prompt": task_description,
                    "session_id": session_id,
                    "start_node": spec.start_node,
                    "seed_id": lineage.seed_id,
                    "seed_dataset": lineage.seed_dataset,
                    "generation_kind": lineage.generation_kind,
                }
                status_url_template = f"{CONTROLLER_URL}/benchmark/{{session_id}}"
                episode_id_key = "episode_id"
                session_id_key = "session_id"
            else:
                session_id = f"eval-{task_id}-{uuid.uuid4().hex[:8]}"

                await _seed_eval_workspace(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    root=ROOT,
                    worker_light_url=WORKER_LIGHT_URL,
                    logger=logger,
                )
                await _preflight_seeded_entry_contract(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    spec=spec,
                    worker_light_url=WORKER_LIGHT_URL,
                    logger=logger,
                )

                url = f"{CONTROLLER_URL}/agent/run"
                payload = {
                    "task": task_description,
                    "agent_name": spec.request_agent_name or agent_name,
                    "start_node": spec.start_node or agent_name,
                    "session_id": session_id,
                    "metadata_vars": lineage.model_dump(exclude_none=True),
                }
                status_url_template = f"{CONTROLLER_URL}/episodes/{{episode_id}}"
                episode_id_key = "episode_id"
                session_id_key = "session_id"

            resp = await client.post(url, json=payload)
            if resp.status_code >= 400:
                log.error(
                    "eval_trigger_failed",
                    status_code=resp.status_code,
                    response_text=resp.text,
                    session_id=session_id,
                )
                stats[agent_name]["total"] += 1
                await _record_hard_check_outcomes(
                    stats=stats,
                    reward_agent_configs=reward_agent_configs,
                    agent_name=agent_name,
                    item=item,
                    success=False,
                    episode_data=last_episode_data,
                    session_id=session_id,
                )
                if run_judge:
                    await _record_judge_outcomes(
                        stats=stats,
                        reward_agent_configs=reward_agent_configs,
                        agent_name=agent_name,
                        item=item,
                        success=False,
                        session_id=session_id,
                        episode_data=last_episode_data,
                    )
                _write_eval_session_metadata(
                    eval_log_key=eval_log_key,
                    payload={
                        "agent_name": agent_name.value,
                        "episode_id": episode_id or None,
                        "eval_mode": spec.mode.value,
                        "session_id": session_id or None,
                        "status": "trigger_failed",
                        "success": False,
                        "task_id": task_id,
                    },
                )
                return
            data = resp.json()
            episode_id = str(data.get(episode_id_key) or data.get("episode_id") or "")
            response_session_id = data.get(session_id_key) or data.get("session_id")
            if response_session_id:
                session_id = str(response_session_id)
                eval_log_key = _resolve_eval_log_key(
                    task_id=task_id, session_id=session_id
                )

            status_url = status_url_template.format(
                session_id=session_id, episode_id=episode_id
            )
            max_attempts = 120
            attempt = 0
            seen_trace_ids = set()
            seen_trace_errors: dict[int, str] = {}
            seen_trace_results: dict[int, str] = {}

            while attempt < max_attempts:
                await asyncio.sleep(5)
                attempt += 1

                try:
                    status_resp = await client.get(status_url)
                    if status_resp.status_code == 200:
                        status_data = status_resp.json()
                        last_episode_data = status_data
                        status = status_data.get("status")

                        traces = status_data.get("traces", [])
                        for trace in sorted(traces, key=lambda t: t.get("id", 0)):
                            trace_id = trace.get("id")
                            readable_line = _format_readable_trace_line(
                                episode=status_data,
                                trace=trace,
                                default_agent_name=readable_agent_name,
                            )
                            if trace_id not in seen_trace_ids:
                                # DEBUG trace lines are always written to logs.
                                # They are also shown on console with DEBUG level.
                                log.debug(
                                    "trace_log",
                                    trace_id=trace_id,
                                    content=trace.get("content"),
                                )
                                # --verbose prints the same lines via INFO.
                                if verbose:
                                    log.info(
                                        "trace_log",
                                        trace_id=trace_id,
                                        content=trace.get("content"),
                                    )
                                if readable_line:
                                    _append_readable_log_line(
                                        readable_line, eval_log_key=eval_log_key
                                    )
                                seen_trace_ids.add(trace_id)

                            error_line = _format_readable_trace_line(
                                episode=status_data,
                                trace=trace,
                                default_agent_name=readable_agent_name,
                                detail_mode="error",
                            )
                            error_text = error_line or ""
                            prior_error_text = seen_trace_errors.get(trace_id, "")
                            if error_text and error_text != prior_error_text:
                                _append_readable_log_line(
                                    error_text, eval_log_key=eval_log_key
                                )
                                seen_trace_errors[trace_id] = error_text

                            result_line = _format_readable_trace_line(
                                episode=status_data,
                                trace=trace,
                                default_agent_name=readable_agent_name,
                                detail_mode="result",
                            )
                            result_text = result_line or ""
                            prior_result_text = seen_trace_results.get(trace_id, "")
                            if result_text and result_text != prior_result_text:
                                _append_readable_log_line(
                                    result_text, eval_log_key=eval_log_key
                                )
                                seen_trace_results[trace_id] = result_text

                        if _requires_expected_review_decision(spec):
                            episode = await _fetch_episode(client, episode_id)
                            last_episode_data = episode
                            missing_traces = _missing_required_traces(
                                spec.required_trace_names, episode
                            )
                            if not missing_traces:
                                review_error = await _review_expectation_error(
                                    item=item,
                                    spec=spec,
                                    session_id=session_id,
                                    worker_light_url=WORKER_LIGHT_URL,
                                )
                                if review_error is None:
                                    log.info(
                                        "eval_review_decision_matched", status=status
                                    )
                                    success = True
                                    break
                                if not _review_artifact_pending(review_error):
                                    log.error(
                                        "eval_failed_expected_decision_mismatch",
                                        session_id=session_id,
                                        error=review_error,
                                    )
                                    break

                        if (
                            status == EpisodeStatus.PLANNED
                            and _planned_counts_as_success(agent_name, spec)
                        ):
                            if agent_name == AgentName.BENCHMARK_PLANNER:
                                episode = await _fetch_episode(client, episode_id)
                                last_episode_data = episode
                                missing_traces = _missing_required_traces(
                                    spec.required_trace_names, episode
                                )
                                if missing_traces:
                                    log.error(
                                        "eval_failed_missing_traces",
                                        missing_traces=missing_traces,
                                        session_id=session_id,
                                    )
                                else:
                                    log.info("eval_planned")
                                    success = True
                                break

                            if (
                                spec.mode == EvalMode.BENCHMARK
                                and not _requires_expected_review_decision(spec)
                            ):
                                log.info("eval_planned_confirming")
                                confirm_url = (
                                    f"{CONTROLLER_URL}/benchmark/{session_id}/confirm"
                                )
                                confirm_resp = await client.post(confirm_url, json={})
                                if confirm_resp.status_code >= 400:
                                    log.error(
                                        "benchmark_confirm_failed",
                                        status_code=confirm_resp.status_code,
                                        response_text=confirm_resp.text,
                                        session_id=session_id,
                                    )
                                    break
                            else:
                                episode = await _fetch_episode(client, episode_id)
                                last_episode_data = episode
                                missing_traces = _missing_required_traces(
                                    spec.required_trace_names, episode
                                )
                                if missing_traces:
                                    log.error(
                                        "eval_failed_missing_traces",
                                        missing_traces=missing_traces,
                                        session_id=session_id,
                                    )
                                else:
                                    completion_error = await _completion_contract_error(
                                        spec=spec,
                                        session_id=session_id,
                                    )
                                    if completion_error:
                                        log.error(
                                            "eval_failed_completion_contract",
                                            session_id=session_id,
                                            error=completion_error,
                                        )
                                    else:
                                        review_error = await _review_expectation_error(
                                            item=item,
                                            spec=spec,
                                            session_id=session_id,
                                            worker_light_url=WORKER_LIGHT_URL,
                                        )
                                        if review_error:
                                            log.error(
                                                "eval_failed_expected_decision_mismatch",
                                                session_id=session_id,
                                                error=review_error,
                                            )
                                        else:
                                            log.info("eval_planned")
                                            success = True
                                break

                        if status == EpisodeStatus.COMPLETED:
                            episode = await _fetch_episode(client, episode_id)
                            last_episode_data = episode
                            missing_traces = _missing_required_traces(
                                spec.required_trace_names, episode
                            )
                            if missing_traces:
                                log.error(
                                    "eval_failed_missing_traces",
                                    missing_traces=missing_traces,
                                    session_id=session_id,
                                )
                            else:
                                completion_error = await _completion_contract_error(
                                    spec=spec,
                                    session_id=session_id,
                                )
                                if completion_error:
                                    log.error(
                                        "eval_failed_completion_contract",
                                        session_id=session_id,
                                        error=completion_error,
                                    )
                                else:
                                    review_error = await _review_expectation_error(
                                        item=item,
                                        spec=spec,
                                        session_id=session_id,
                                        worker_light_url=WORKER_LIGHT_URL,
                                    )
                                    if review_error:
                                        log.error(
                                            "eval_failed_expected_decision_mismatch",
                                            session_id=session_id,
                                            error=review_error,
                                        )
                                    else:
                                        log.info("eval_completed")
                                        success = True
                            break

                        if status == EpisodeStatus.FAILED:
                            episode = await _fetch_episode(client, episode_id)
                            last_episode_data = episode
                            missing_traces = _missing_required_traces(
                                spec.required_trace_names, episode
                            )
                            if missing_traces:
                                log.error(
                                    "eval_failed_missing_traces",
                                    missing_traces=missing_traces,
                                    session_id=session_id,
                                )
                            else:
                                review_error = await _review_expectation_error(
                                    item=item,
                                    spec=spec,
                                    session_id=session_id,
                                    worker_light_url=WORKER_LIGHT_URL,
                                )
                                if review_error:
                                    log.error(
                                        "eval_failed",
                                        session_id=session_id,
                                        error=review_error,
                                    )
                                elif (
                                    item.expected_decision is not None
                                    and item.expected_decision
                                    != ReviewDecision.APPROVED
                                ):
                                    log.info("eval_failed_as_expected")
                                    success = True
                                else:
                                    log.error("eval_failed", session_id=session_id)
                            break

                        if status == EpisodeStatus.CANCELLED:
                            log.warning("eval_cancelled")
                            break

                        if attempt % 6 == 0:
                            log.info(
                                "eval_still_running", status=status, attempt=attempt
                            )
                    else:
                        log.warning(
                            "eval_status_check_failed",
                            status_code=status_resp.status_code,
                        )
                except Exception:
                    log.exception("eval_status_check_exception")

            if attempt >= max_attempts:
                log.warning("eval_timeout", max_attempts=max_attempts)

            agent_stats = stats[agent_name]
            eval_cost_usd = _extract_episode_cost_usd(last_episode_data)
            agent_stats["total"] += 1
            if eval_cost_usd is not None:
                agent_stats["total_cost_usd"] += eval_cost_usd
                agent_stats["costed_cases"] += 1
            if success:
                agent_stats["success"] += 1

                worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
                handler = METRIC_HANDLERS.get(agent_name)
                try:
                    if handler:
                        await handler(worker, session_id, agent_stats)
                finally:
                    await worker.aclose()

            hard_checks_passed = await _record_hard_check_outcomes(
                stats=stats,
                reward_agent_configs=reward_agent_configs,
                agent_name=agent_name,
                item=item,
                success=success,
                episode_data=last_episode_data,
                session_id=session_id,
            )
            reviewer_episodes: list[dict[str, Any]] = []
            if (
                run_judge
                and run_reviewers_with_judge
                and success
                and hard_checks_passed is True
            ):
                reviewer_episodes = await _run_reviewer_chain_for_judge(
                    client=client,
                    agent_name=agent_name,
                    session_id=session_id,
                    task_description=task_description,
                    lineage=lineage,
                    log=log,
                )

            if run_judge:
                await _record_judge_outcomes(
                    stats=stats,
                    reward_agent_configs=reward_agent_configs,
                    agent_name=agent_name,
                    item=item,
                    success=success,
                    session_id=session_id,
                    episode_data=last_episode_data,
                    extra_episodes=reviewer_episodes,
                )
        except asyncio.CancelledError:
            log.warning(
                "eval_run_interrupted",
                session_id=session_id or None,
                episode_id=episode_id or None,
            )
            if episode_id:
                await asyncio.shield(
                    _request_episode_interrupt(
                        episode_id,
                        log=log,
                    )
                )
            raise
        except Exception:
            log.exception("controller_request_failed")
            stats[agent_name]["total"] += 1
            await _record_hard_check_outcomes(
                stats=stats,
                reward_agent_configs=reward_agent_configs,
                agent_name=agent_name,
                item=item,
                success=False,
                episode_data=last_episode_data,
                session_id=session_id,
            )
            if run_judge:
                await _record_judge_outcomes(
                    stats=stats,
                    reward_agent_configs=reward_agent_configs,
                    agent_name=agent_name,
                    item=item,
                    success=False,
                    session_id=session_id,
                    episode_data=last_episode_data,
                )
            _write_eval_session_metadata(
                eval_log_key=eval_log_key,
                payload={
                    "agent_name": agent_name.value,
                    "episode_id": episode_id or None,
                    "eval_mode": spec.mode.value,
                    "session_id": session_id or None,
                    "status": "controller_request_failed",
                    "success": False,
                    "task_id": task_id,
                },
            )
            return

    _write_eval_session_metadata(
        eval_log_key=eval_log_key,
        payload={
            "agent_name": agent_name.value,
            "episode_id": episode_id or None,
            "eval_mode": spec.mode.value,
            "session_id": session_id or None,
            "status": "completed",
            "success": success,
            "task_id": task_id,
        },
    )


async def main():
    parser = argparse.ArgumentParser(description="Run Agent Evals")

    available_agents = list(AGENT_SPECS.keys())

    parser.add_argument(
        "--agent",
        type=str,
        default="all",
        help="Agent to evaluate",
    )
    parser.add_argument(
        "--limit", type=int, default=0, help="Limit number of eval items per agent"
    )
    parser.add_argument(
        "--task-id",
        action="append",
        default=None,
        help=(
            "Run only specific task IDs. Supports a single ID, repeated flags, "
            "comma-separated values, or list syntax like [id-1,id-2]."
        ),
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Print backend traces during polling"
    )
    parser.add_argument(
        "--run-judge",
        action="store_true",
        help="Record judge/checklist pass rates from reward_config judge_evaluation",
    )
    parser.add_argument(
        "--run-reviewers-with-judge",
        action="store_true",
        help=(
            "When --run-judge is enabled, run reviewer stages after all hard checks "
            "pass to populate reviewer checklist metrics."
        ),
    )
    parser.add_argument(
        "--concurrency",
        type=int,
        default=5,
        help=(
            "Max number of eval tasks to run concurrently "
            "(default: 5 for parallel performance)"
        ),
    )
    parser.add_argument(
        "--no-rate-limit", action="store_true", help="Disable the 50 RPM rate limit"
    )
    parser.add_argument(
        "--skip-env-up", action="store_true", help="Skip running scripts/env_up.sh"
    )
    parser.add_argument(
        "--log-level",
        "--log_level",
        type=str,
        default="DEBUG",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Log level for the console output (default: DEBUG)",
    )
    args = parser.parse_args()
    selected_task_ids = _parse_task_id_filters(args.task_id)

    if args.task_id and not selected_task_ids:
        parser.error("--task-id provided but no valid task IDs were parsed")

    if args.run_reviewers_with_judge and not args.run_judge:
        parser.error("--run-reviewers-with-judge requires --run-judge")

    # Configure logging to directory (logs/evals/runs/<run_id>)
    evals_root = ROOT / "logs" / "evals"
    runs_root = evals_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    run_name = f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    log_dir = runs_root / run_name
    suffix = 1
    while log_dir.exists():
        log_dir = runs_root / f"{run_name}_{suffix}"
        suffix += 1
    log_dir.mkdir(parents=True, exist_ok=True)

    now = time.time()
    cutoff_seconds = 24 * 60 * 60
    for old_run in runs_root.glob("run_*"):
        try:
            if now - old_run.stat().st_mtime > cutoff_seconds:
                if old_run.is_dir():
                    shutil.rmtree(old_run, ignore_errors=True)
                else:
                    old_run.unlink(missing_ok=True)
        except OSError:
            continue

    log_file = log_dir / "run_evals.log"
    readable_log_file = log_dir / "readable_agent_logs.log"
    session_log_root = log_dir / "sessions"
    session_log_root.mkdir(parents=True, exist_ok=True)

    # Symlink for latest run
    current_link = evals_root / "current"
    if current_link.exists() or current_link.is_symlink():
        with contextlib.suppress(Exception):
            current_link.unlink()
    with contextlib.suppress(Exception):
        current_link.symlink_to(log_dir.relative_to(evals_root))

    latest_log = evals_root / "latest.log"
    if latest_log.exists() or latest_log.is_symlink():
        with contextlib.suppress(Exception):
            latest_log.unlink()
    with contextlib.suppress(Exception):
        latest_log.symlink_to(Path("current") / "run_evals.log")

    if readable_log_file.exists():
        with contextlib.suppress(Exception):
            readable_log_file.unlink()
    readable_log_file.touch()

    os.environ["LOG_DIR"] = str(log_dir)
    os.environ["SESSION_LOG_ROOT"] = str(session_log_root)
    os.environ["EXTRA_DEBUG_LOG"] = str(log_file)
    os.environ["LOG_LEVEL"] = args.log_level
    configure_logging("evals")

    global logger
    global READABLE_AGENT_LOG_FILE
    global SESSION_LOG_ROOT
    logger = get_logger(__name__)
    READABLE_AGENT_LOG_FILE = readable_log_file
    SESSION_LOG_ROOT = session_log_root

    print(f"Logging to: {log_file} (and symlinked at {latest_log})")
    print(f"Readable agent logs: {readable_log_file}")
    print(f"Per-eval session logs: {session_log_root}")
    logger.info("eval_run_start", log_dir=str(log_dir))

    start_time = time.time()
    print(f"Agent Evals Started at: {time.ctime(start_time)}")

    # Run env_up.sh before checking health or starting evaluations
    if not args.skip_env_up:
        logger.info("env_up_start")
        try:
            env_up_path = ROOT / "scripts" / "env_up.sh"
            # Set LOG_DIR relative to ROOT for env_up.sh
            env_up_log_dir = log_dir.relative_to(ROOT)
            result = subprocess.run(
                [str(env_up_path)],
                check=True,
                capture_output=True,
                text=True,
                env={
                    **os.environ,
                    "LOG_DIR": str(env_up_log_dir),
                    "SKIP_LOG_ARCHIVE": "1",
                },
            )
            # Dump output into the log file at DEBUG level
            for line in result.stdout.splitlines():
                if line.strip():
                    logger.debug("env_up_output", line=line)
            logger.info("env_up_completed")

            # Update symlinks in logs/ root to point to eval logs instead of manual_run
            logs_root = ROOT / "logs"
            links = [
                ("evals/current/controller.log", "controller.log"),
                ("evals/current/controller_debug.log", "controller_debug.log"),
                ("evals/current/worker_light.log", "worker_light.log"),
                ("evals/current/worker_heavy.log", "worker_heavy.log"),
                ("evals/current/worker_light_debug.log", "worker_light_debug.log"),
                ("evals/current/worker_heavy_debug.log", "worker_heavy_debug.log"),
                ("evals/current/temporal_worker.log", "temporal_worker.log"),
                (
                    "evals/current/temporal_worker_debug.log",
                    "temporal_worker_debug.log",
                ),
                ("evals/current/frontend.log", "frontend.log"),
                ("evals/current/run_evals.log", "run_evals.log"),
                ("evals/current/readable_agent_logs.log", "readable_agent_logs.log"),
            ]
            for target, link_name in links:
                link_path = logs_root / link_name
                try:
                    if link_path.exists() or link_path.is_symlink():
                        link_path.unlink()
                    link_path.symlink_to(target)
                except Exception:
                    pass

        except subprocess.CalledProcessError as e:
            logger.warning("env_up_failed", stdout=e.stdout, stderr=e.stderr)
            print(f"ERROR: env_up.sh failed with exit code {e.returncode}")
            print(f"Check logs for details: {log_file}")
            sys.exit(1)
        except Exception:
            logger.exception("env_up_exception")
            sys.exit(1)

    # Rate limiter setup (50 RPM)
    rate = Rate(50, Duration.MINUTE)
    limiter = Limiter(rate)

    requested_agent = args.agent
    if requested_agent != "all":
        try:
            requested_agent = AgentName(requested_agent)
        except ValueError:
            logger.error("unknown_agent", agent=requested_agent, session_id="eval")
            logger.info("available_agents")
            for name in available_agents:
                logger.info("agent", name=name)
            sys.exit(1)

    if requested_agent != "all" and requested_agent not in AGENT_SPECS:
        logger.error("agent_not_in_specs", agent=requested_agent, session_id="eval")
        sys.exit(1)

    # Check health
    logger.info("controller_health_check_start", url=CONTROLLER_URL)
    try:
        await _wait_for_controller_ready()
    except Exception:
        logger.exception("controller_unreachable", url=CONTROLLER_URL)
        sys.exit(1)

    logger.info("worker_health_check_start", url=WORKER_LIGHT_URL)
    try:
        await _wait_for_worker_ready()
    except Exception:
        logger.exception("worker_unreachable", url=WORKER_LIGHT_URL)
        sys.exit(1)

    dataset_roots = [
        Path(__file__).parent / "datasets",
        ROOT / "dataset" / "data" / "seed" / "role_based",
    ]
    datasets = {}
    agents_to_run = available_agents if requested_agent == "all" else [requested_agent]
    reward_agent_configs = _load_agent_reward_configs()

    stats = {
        agent: {
            "total": 0,
            "success": 0,
            "total_cost_usd": 0.0,
            "costed_cases": 0,
            "electrical_validity_rate": 0.0,
            "wire_integrity_rate": 0.0,
            "power_efficiency_score": 0.0,
            "hard_checks": {},
            "judge_checks": {},
        }
        for agent in agents_to_run
    }

    for agent in agents_to_run:
        agent_val = agent.value if isinstance(agent, AgentName) else agent
        json_path = next(
            (
                root / f"{agent_val}.json"
                for root in dataset_roots
                if (root / f"{agent_val}.json").exists()
            ),
            None,
        )
        if json_path is not None:
            with json_path.open() as f:
                try:
                    data = json.load(f)
                    if selected_task_ids:
                        data = [
                            item for item in data if item["id"] in selected_task_ids
                        ]
                    if args.limit > 0:
                        data = data[: args.limit]
                    seed_dataset = json_path.relative_to(ROOT)
                    datasets[agent] = [
                        EvalDatasetItem.model_validate(
                            {**item_raw, "seed_dataset": seed_dataset}
                        )
                        for item_raw in data
                    ]
                    if _requires_expected_review_decision(AGENT_SPECS[agent]):
                        missing_expectations = [
                            item.id
                            for item in datasets[agent]
                            if item.expected_decision is None
                        ]
                        if missing_expectations:
                            raise ValueError(
                                f"{agent.value} eval rows missing expected_decision: "
                                + ", ".join(missing_expectations)
                            )
                except json.JSONDecodeError:
                    logger.warning("dataset_json_decode_failed", path=str(json_path))
        else:
            logger.warning(
                "dataset_missing",
                agent=agent,
                searched_roots=[str(p) for p in dataset_roots],
            )

    tasks = []
    for agent, dataset in datasets.items():
        logger.info("agent_evals_start", agent=agent, count=len(dataset))
        for item in dataset:
            tasks.append((item, agent))

    if tasks:
        semaphore = asyncio.Semaphore(max(1, args.concurrency))

        async def _guarded(item: EvalDatasetItem, agent: AgentName):
            async with semaphore:
                if not args.no_rate_limit:
                    await asyncio.to_thread(limiter.try_acquire, "eval")
                await run_single_eval(
                    item,
                    agent,
                    stats,
                    reward_agent_configs,
                    verbose=args.verbose,
                    run_judge=args.run_judge,
                    run_reviewers_with_judge=args.run_reviewers_with_judge,
                )

        await asyncio.gather(*(_guarded(item, agent) for item, agent in tasks))
    else:
        logger.warning("no_tasks_to_run")

    logger.info("evaluation_report_start")

    total_pass = 0
    total_count = 0
    for agent, s in stats.items():
        if s["total"] == 0:
            continue

        success_rate = (s["success"] / s["total"]) * 100
        total_pass += s["success"]
        total_count += s["total"]

        log_report = logger.bind(agent=agent)
        log_report.info(
            "agent_summary",
            task_success_rate_pct=round(success_rate, 1),
            successful_tasks=s["success"],
            total_tasks=s["total"],
        )
        if s["costed_cases"] > 0:
            log_report.info(
                "agent_cost_summary",
                total_cost_usd=round(s["total_cost_usd"], 6),
                costed_cases=s["costed_cases"],
                avg_cost_per_case_usd=round(
                    s["total_cost_usd"] / s["costed_cases"],
                    6,
                ),
            )

        if agent in METRIC_HANDLERS and s["success"] > 0:
            log_report.info(
                "agent_electronics_metrics",
                electrical_validity_rate_pct=round(
                    (s["electrical_validity_rate"] / s["success"]) * 100, 1
                ),
                wire_integrity_rate_pct=round(
                    (s["wire_integrity_rate"] / s["success"]) * 100, 1
                ),
                avg_power_efficiency_score=round(
                    s["power_efficiency_score"] / s["success"], 2
                ),
            )

    overall = (total_pass / total_count * 100) if total_count else 0.0
    logger.info(
        "overall_summary",
        overall_pass_rate_pct=round(overall, 1),
        passed_tasks=total_pass,
        total_tasks=total_count,
    )
    total_cost_usd = sum(float(s["total_cost_usd"]) for s in stats.values())
    total_costed_cases = sum(int(s["costed_cases"]) for s in stats.values())
    if total_costed_cases > 0:
        logger.info(
            "overall_cost_summary",
            total_cost_usd=round(total_cost_usd, 6),
            costed_cases=total_costed_cases,
            avg_cost_per_case_usd=round(total_cost_usd / total_costed_cases, 6),
        )

    agent_ran_cases: dict[AgentName, list[str]] = {
        agent: [item.id for item in datasets.get(agent, [])] for agent in agents_to_run
    }

    def _build_check_report(check_key: str, payload_key: str) -> dict[str, Any]:
        report: dict[str, Any] = {}
        for agent, agent_stats in stats.items():
            check_stats = agent_stats.get(check_key) or {}
            agent_key = agent.value if isinstance(agent, AgentName) else str(agent)
            ran_cases = agent_ran_cases.get(agent, [])
            checks_payload: dict[str, Any] = {}
            for check_name, raw in check_stats.items():
                total = int(raw.get("total", 0))
                passed = int(raw.get("passed", 0))
                failed_seeds = sorted(set(raw.get("failed_seeds", [])))
                pass_rate = round((passed / total) * 100) if total else 0
                checks_payload[check_name] = {
                    "pass_rate": f"{pass_rate}%",
                    "passed": passed,
                    "total": total,
                    "failed_seeds": failed_seeds,
                }
            report[agent_key] = {
                "ran_cases": ran_cases,
                payload_key: checks_payload,
            }
        return report

    hard_check_report = _build_check_report("hard_checks", "hard_checks")

    hard_check_report_path = log_dir / "hard_check_pass_rates.yaml"
    with hard_check_report_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(
            hard_check_report,
            handle,
            sort_keys=True,
            allow_unicode=False,
            default_flow_style=False,
        )

    print("Hard check pass rates:")
    print(
        yaml.safe_dump(
            hard_check_report,
            sort_keys=True,
            allow_unicode=False,
            default_flow_style=False,
        ).rstrip()
    )
    print(f"Hard check report: {hard_check_report_path}")
    logger.info(
        "hard_check_report_written",
        path=str(hard_check_report_path),
        agents=list(hard_check_report.keys()),
    )
    if total_costed_cases > 0:
        avg_cost = total_cost_usd / total_costed_cases
        print(
            "Average eval case cost: "
            f"${avg_cost:.6f} "
            f"(total ${total_cost_usd:.6f} across {total_costed_cases} costed cases)"
        )
    else:
        print("Average eval case cost: unavailable (no cost telemetry recorded)")

    if args.run_judge:
        judge_report = _build_check_report("judge_checks", "judge_checks")
        judge_report_path = log_dir / "judge_pass_rates.yaml"
        with judge_report_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(
                judge_report,
                handle,
                sort_keys=True,
                allow_unicode=False,
                default_flow_style=False,
            )

        print("Judge/checklist pass rates:")
        print(
            yaml.safe_dump(
                judge_report,
                sort_keys=True,
                allow_unicode=False,
                default_flow_style=False,
            ).rstrip()
        )
        print(f"Judge report: {judge_report_path}")
        logger.info(
            "judge_report_written",
            path=str(judge_report_path),
            agents=list(judge_report.keys()),
            reviewers_enabled=args.run_reviewers_with_judge,
        )

    finish_time = time.time()
    duration = finish_time - start_time
    print(f"Agent Evals Finished at: {time.ctime(finish_time)}")
    print(f"Total Duration: {duration:.2f}s")
    logger.info("eval_run_finished", duration_s=round(duration, 2))


def run_cli() -> None:
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Agent evals interrupted.")


if __name__ == "__main__":
    run_cli()
