import argparse
import asyncio
import contextlib
import json
import os
import re
import shutil
import subprocess
import sys
import time
import uuid
from datetime import datetime
from pathlib import Path
from threading import Lock
from typing import Any, Literal

import httpx
from dotenv import load_dotenv
from pydantic import BaseModel, ConfigDict, Field, model_validator
from pyrate_limiter import Duration, Limiter, Rate

# Ensure repository root is importable when script is executed as a file.
ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controller.agent.node_entry_validation import (  # noqa: E402
    BENCHMARK_CODER_HANDOVER_CHECK,
    BENCHMARK_REVIEWER_HANDOVER_CHECK,
    ELECTRONICS_REVIEWER_HANDOVER_CHECK,
    ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK,
    ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK,
    ValidationGraph,
    benchmark_coder_handover_custom_check_from_session_id,
    build_benchmark_node_contracts,
    build_engineer_node_contracts,
    evaluate_node_entry_contract,
    plan_reviewer_handover_custom_check_from_session_id,
    reviewer_handover_custom_check_from_session_id,
)
from controller.agent.review_handover import validate_reviewer_handover  # noqa: E402
from controller.clients.worker import WorkerClient  # noqa: E402
from shared.enums import (  # noqa: E402
    AgentName,
    EpisodeStatus,
    EvalMode,
    GenerationKind,
    ReviewDecision,
    SeedMatchMethod,
)
from shared.logging import configure_logging, get_logger  # noqa: E402
from shared.models.schemas import EpisodeMetadata, ReviewFrontmatter  # noqa: E402
from shared.utils.evaluation import analyze_electronics_metrics  # noqa: E402
from worker_heavy.utils.file_validation import validate_review_frontmatter  # noqa: E402

load_dotenv()
# Logging will be configured in main() to support redirection to logs/evals
logger = get_logger(__name__)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
READABLE_AGENT_LOG_FILE: Path | None = None
_READABLE_AGENT_LOG_LOCK = Lock()


class AgentEvalSpec(BaseModel):
    """Runtime details for an eval agent type."""

    mode: EvalMode  # benchmark | agent | git
    request_agent_name: AgentName | None = None
    required_trace_names: tuple[AgentName, ...] = ()
    start_node: AgentName | None = None
    required_reviewer_handover_manifest: str | None = None
    required_reviewer_stage: str | None = None
    materialize_reviewer_handover: bool = False
    review_filename_prefix: str | None = None


class EvalDatasetItem(BaseModel):
    id: str
    task: str
    seed_dataset: Path | None = None
    seed_artifact_dir: Path | None = None
    seed_files: dict[str, str] | None = None
    git_eval: "GitEvalConfig | None" = None
    expected_decision: ReviewDecision | None = None

    model_config = ConfigDict(extra="allow")


class GitStatusExpectation(BaseModel):
    branch: str | None = None
    is_dirty: bool | None = None
    is_merging: bool | None = None
    conflicts: list[str] | None = None


class GitConflictResolutionStep(BaseModel):
    file_path: str
    strategy: Literal["ours", "theirs"]


class GitEvalConfig(BaseModel):
    setup_commands: list[str] = Field(default_factory=list)
    commit_message: str | None = None
    expect_commit_hash: bool | None = None
    resolve_conflicts: list[GitConflictResolutionStep] = Field(default_factory=list)
    abort_merge: bool = False
    merge_complete_message: str | None = None
    expected_status: GitStatusExpectation | None = None

    @model_validator(mode="after")
    def validate_merge_actions(self) -> "GitEvalConfig":
        if self.abort_merge and self.merge_complete_message is not None:
            raise ValueError(
                "git_eval.abort_merge and git_eval.merge_complete_message are mutually exclusive"
            )
        return self


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


def _append_readable_log_line(line: str) -> None:
    if READABLE_AGENT_LOG_FILE is None:
        return

    with _READABLE_AGENT_LOG_LOCK:
        READABLE_AGENT_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        with READABLE_AGENT_LOG_FILE.open("a", encoding="utf-8") as handle:
            handle.write(line.rstrip() + "\n")


AGENT_SPECS: dict[AgentName, AgentEvalSpec] = {
    # Benchmark graph roles
    AgentName.BENCHMARK_PLANNER: AgentEvalSpec(
        mode=EvalMode.BENCHMARK,
        request_agent_name=AgentName.BENCHMARK_PLANNER,
        required_trace_names=(AgentName.BENCHMARK_PLANNER,),
        start_node=AgentName.BENCHMARK_PLANNER,
    ),
    AgentName.BENCHMARK_CODER: AgentEvalSpec(
        mode=EvalMode.BENCHMARK,
        request_agent_name=AgentName.BENCHMARK_CODER,
        required_trace_names=(AgentName.BENCHMARK_CODER,),
        start_node=AgentName.BENCHMARK_CODER,
    ),
    AgentName.BENCHMARK_REVIEWER: AgentEvalSpec(
        mode=EvalMode.BENCHMARK,
        request_agent_name=AgentName.BENCHMARK_REVIEWER,
        required_trace_names=(AgentName.BENCHMARK_REVIEWER,),
        start_node=AgentName.BENCHMARK_REVIEWER,
        review_filename_prefix="benchmark-review-round",
    ),
    # Mechanical engineering roles
    AgentName.ENGINEER_PLANNER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_PLANNER,
        required_trace_names=(AgentName.ENGINEER_PLANNER,),
        start_node=AgentName.ENGINEER_PLANNER,
    ),
    AgentName.ENGINEER_CODER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_CODER,
        required_trace_names=(AgentName.ENGINEER_CODER,),
        start_node=AgentName.ENGINEER_CODER,
        required_reviewer_handover_manifest=(
            ".manifests/engineering_execution_review_manifest.json"
        ),
        required_reviewer_stage="engineering_execution_reviewer",
        materialize_reviewer_handover=True,
    ),
    AgentName.ENGINEER_PLAN_REVIEWER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_CODER,
        required_trace_names=(AgentName.ENGINEER_PLAN_REVIEWER,),
        start_node=AgentName.ENGINEER_PLAN_REVIEWER,
        review_filename_prefix="engineering-plan-review-round",
    ),
    AgentName.ENGINEER_EXECUTION_REVIEWER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_CODER,
        required_trace_names=(AgentName.ENGINEER_EXECUTION_REVIEWER,),
        start_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        review_filename_prefix="engineering-execution-review-round",
    ),
    # Electrical engineering roles inside the unified engineer graph
    AgentName.ELECTRONICS_PLANNER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_PLANNER,
        required_trace_names=(AgentName.ELECTRONICS_PLANNER,),
        start_node=AgentName.ELECTRONICS_PLANNER,
    ),
    AgentName.ELECTRONICS_REVIEWER: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.ENGINEER_CODER,
        required_trace_names=(AgentName.ELECTRONICS_REVIEWER,),
        start_node=AgentName.ELECTRONICS_REVIEWER,
        review_filename_prefix="electronics-review-round",
    ),
    AgentName.COTS_SEARCH: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.COTS_SEARCH,
        required_trace_names=(AgentName.COTS_SEARCH,),
    ),
    # Sidecars
    AgentName.SKILL_AGENT: AgentEvalSpec(
        mode=EvalMode.AGENT,
        request_agent_name=AgentName.SKILL_AGENT,
        required_trace_names=(AgentName.SKILL_AGENT,),
    ),
    AgentName.GIT_AGENT: AgentEvalSpec(
        mode=EvalMode.GIT, request_agent_name=AgentName.GIT_AGENT
    ),
}


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


def _resolve_seed_artifact_dir(item: EvalDatasetItem) -> Path | None:
    if item.seed_artifact_dir is None:
        return None

    artifact_dir = Path(item.seed_artifact_dir)
    if artifact_dir.is_absolute():
        return artifact_dir

    repo_relative = ROOT / artifact_dir
    if repo_relative.exists():
        return repo_relative

    if item.seed_dataset is not None:
        dataset_relative = (ROOT / item.seed_dataset).parent / artifact_dir
        if dataset_relative.exists():
            return dataset_relative

    return repo_relative


async def _seed_eval_workspace(
    *,
    item: EvalDatasetItem,
    session_id: str,
    agent_name: AgentName,
) -> None:
    artifact_dir = _resolve_seed_artifact_dir(item)
    inline_files = item.seed_files or {}
    if artifact_dir is None and not inline_files:
        return

    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    seeded_paths: list[str] = []
    try:
        if artifact_dir is not None:
            if not artifact_dir.exists():
                raise FileNotFoundError(
                    f"Seed artifact directory not found: {artifact_dir}"
                )
            for path in sorted(p for p in artifact_dir.rglob("*") if p.is_file()):
                rel_path = path.relative_to(artifact_dir).as_posix()
                raw_bytes = path.read_bytes()
                try:
                    content = raw_bytes.decode("utf-8")
                except UnicodeDecodeError:
                    await worker.upload_file(
                        rel_path,
                        raw_bytes,
                        bypass_agent_permissions=True,
                    )
                else:
                    await worker.write_file(
                        rel_path,
                        content,
                        overwrite=True,
                        bypass_agent_permissions=True,
                    )
                seeded_paths.append(rel_path)

        for rel_path, content in inline_files.items():
            await worker.write_file(
                rel_path,
                content,
                overwrite=True,
                bypass_agent_permissions=True,
            )
            seeded_paths.append(rel_path)
    finally:
        await worker.aclose()

    logger.info(
        "eval_seed_workspace_applied",
        session_id=session_id,
        agent_name=agent_name,
        seed_file_count=len(seeded_paths),
        seeded_paths=seeded_paths,
    )


async def _preflight_seeded_entry_contract(
    *,
    item: EvalDatasetItem,
    session_id: str,
    agent_name: AgentName,
    spec: AgentEvalSpec,
) -> None:
    if item.seed_artifact_dir is None and not item.seed_files:
        return

    if spec.mode == EvalMode.BENCHMARK:
        target_node = spec.start_node or agent_name
        contracts = build_benchmark_node_contracts()
        graph = ValidationGraph.BENCHMARK
    elif spec.mode == EvalMode.AGENT:
        target_node = spec.start_node or agent_name
        contracts = build_engineer_node_contracts()
        graph = ValidationGraph.ENGINEER
    else:
        return

    contract = contracts.get(target_node)
    if contract is None:
        return

    custom_checks = {
        BENCHMARK_CODER_HANDOVER_CHECK: (
            lambda *, contract, state: (
                benchmark_coder_handover_custom_check_from_session_id(
                    session_id=session_id,
                    custom_objectives=None,
                )
            )
        ),
        BENCHMARK_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Benchmark",
                manifest_path=".manifests/benchmark_review_manifest.json",
                expected_stage="benchmark_reviewer",
            )
        ),
        ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: (
                plan_reviewer_handover_custom_check_from_session_id(
                    session_id=session_id,
                )
            )
        ),
        ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Execution",
                manifest_path=".manifests/engineering_execution_review_manifest.json",
                expected_stage="engineering_execution_reviewer",
            )
        ),
        ELECTRONICS_REVIEWER_HANDOVER_CHECK: (
            lambda *, contract, state: reviewer_handover_custom_check_from_session_id(  # noqa: ARG005
                session_id=session_id,
                reviewer_label="Electronics",
                manifest_path=".manifests/electronics_review_manifest.json",
                expected_stage="electronics_reviewer",
            )
        ),
    }

    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        result = await evaluate_node_entry_contract(
            contract=contract,
            state={
                "task": item.task,
                "episode_id": session_id,
                "session": {
                    "session_id": session_id,
                    "custom_objectives": None,
                },
            },
            artifact_exists=worker.exists,
            graph=graph,
            custom_checks=custom_checks,
            integration_mode=True,
        )
    finally:
        await worker.aclose()

    if result.ok:
        logger.info(
            "eval_seed_entry_preflight_passed",
            session_id=session_id,
            agent_name=agent_name,
            target_node=target_node,
        )
        return

    errors = [error.model_dump(mode="json") for error in result.errors]
    raise ValueError(
        f"Seeded entry contract invalid for {target_node.value}: "
        + "; ".join(f"{error['code']}: {error['message']}" for error in errors)
    )


def _episode_terminal(status: str | None) -> bool:
    if not status:
        return False
    return status in {
        EpisodeStatus.COMPLETED,
        EpisodeStatus.FAILED,
        EpisodeStatus.CANCELLED,
    }


def _planned_counts_as_success(agent_name: AgentName, spec: AgentEvalSpec) -> bool:
    if spec.mode == EvalMode.BENCHMARK:
        return True
    return agent_name in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.BENCHMARK_PLANNER,
    }


def _requires_expected_review_decision(spec: AgentEvalSpec) -> bool:
    return spec.review_filename_prefix is not None


async def _load_latest_review_frontmatter(
    *,
    worker: WorkerClient,
    review_filename_prefix: str,
    session_id: str,
) -> tuple[ReviewFrontmatter | None, str | None]:
    pattern = re.compile(rf"^{re.escape(review_filename_prefix)}-(\d+)\.md$")
    try:
        entries = await worker.list_files("reviews", bypass_agent_permissions=True)
    except FileNotFoundError:
        return None, "reviews/ directory not found"
    except Exception as exc:
        return None, f"failed to list reviews/: {exc}"

    latest_name: str | None = None
    latest_round = -1
    for entry in entries:
        if entry.is_dir:
            continue
        match = pattern.fullmatch(entry.name)
        if match is None:
            continue
        round_number = int(match.group(1))
        if round_number > latest_round:
            latest_round = round_number
            latest_name = entry.name

    if latest_name is None:
        return (
            None,
            "no persisted review file matching "
            f"reviews/{review_filename_prefix}-<n>.md",
        )

    review_path = f"reviews/{latest_name}"
    try:
        content = await worker.read_file(review_path, bypass_agent_permissions=True)
    except Exception as exc:
        return None, f"failed to read {review_path}: {exc}"

    is_valid, review_data = validate_review_frontmatter(
        content,
        cad_agent_refused=True,
        session_id=session_id,
    )
    if not is_valid:
        details = (
            ", ".join(review_data)
            if isinstance(review_data, list)
            else str(review_data)
        )
        return None, f"invalid review frontmatter in {review_path}: {details}"
    if not isinstance(review_data, ReviewFrontmatter):
        return None, f"unexpected review frontmatter payload for {review_path}"

    return review_data, None


async def _review_expectation_error(
    *,
    item: EvalDatasetItem,
    spec: AgentEvalSpec,
    session_id: str,
) -> str | None:
    expected_decision = item.expected_decision
    review_filename_prefix = spec.review_filename_prefix
    if expected_decision is None or review_filename_prefix is None:
        return None

    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        review_frontmatter, review_error = await _load_latest_review_frontmatter(
            worker=worker,
            review_filename_prefix=review_filename_prefix,
            session_id=session_id,
        )
    finally:
        await worker.aclose()

    if review_error:
        return review_error
    if review_frontmatter is None:
        return "missing persisted review frontmatter"
    if review_frontmatter.decision != expected_decision:
        return (
            "expected review decision "
            f"{expected_decision.value}, got {review_frontmatter.decision.value}"
        )
    return None


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
        f"Controller did not become healthy within {timeout_seconds}s. last_error={last_error}"
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
        f"Worker did not become healthy within {timeout_seconds}s. last_error={last_error}"
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


async def _run_git_eval(
    item: EvalDatasetItem, stats: dict[AgentName, Any], agent_name: AgentName
):
    task_id = item.id
    log = logger.bind(task_id=task_id, agent_name=agent_name, eval_mode=EvalMode.GIT)
    log.info("eval_start")

    git_eval = item.git_eval or GitEvalConfig()
    session_id = f"eval-git-{task_id}-{uuid.uuid4().hex[:8]}"
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
            result = await worker.execute_command(command, timeout=60)
            if result.timed_out:
                failure_reason = f"setup command timed out: {command}"
                break
            if result.exit_code != 0:
                stderr = _truncate_text(result.stderr or result.stdout or "")
                failure_reason = (
                    f"setup command failed with exit_code={result.exit_code}: "
                    f"{command} ({stderr})"
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

    agent_stats = stats[agent_name]
    agent_stats["total"] += 1
    if success:
        agent_stats["success"] += 1


async def run_single_eval(
    item: EvalDatasetItem,
    agent_name: AgentName,
    stats: dict[AgentName, Any],
    verbose: bool = False,
):
    """
    Runs a single evaluation task for a specific agent type.
    """
    spec = AGENT_SPECS[agent_name]

    if spec.mode == EvalMode.GIT:
        await _run_git_eval(item, stats, agent_name)
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

    async with httpx.AsyncClient(timeout=60.0) as client:
        try:
            if spec.mode == EvalMode.BENCHMARK:
                benchmark_session_id = uuid.uuid4()
                session_id = str(benchmark_session_id)
                await _seed_eval_workspace(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                )
                await _preflight_seeded_entry_contract(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    spec=spec,
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
                )
                await _preflight_seeded_entry_contract(
                    item=item,
                    session_id=session_id,
                    agent_name=agent_name,
                    spec=spec,
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
                return
            data = resp.json()
            episode_id = str(data.get(episode_id_key) or data.get("episode_id") or "")
            response_session_id = data.get(session_id_key) or data.get("session_id")
            if response_session_id:
                session_id = str(response_session_id)

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
                                # Restate traces at DEBUG level so they go to the log file.
                                # They will also appear on console if --log-level DEBUG is used.
                                log.debug(
                                    "trace_log",
                                    trace_id=trace_id,
                                    content=trace.get("content"),
                                )
                                # If verbose is set, log at INFO so they show up on console regardless of log level.
                                if verbose:
                                    log.info(
                                        "trace_log",
                                        trace_id=trace_id,
                                        content=trace.get("content"),
                                    )
                                if readable_line:
                                    _append_readable_log_line(readable_line)
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
                                _append_readable_log_line(error_text)
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
                                _append_readable_log_line(result_text)
                                seen_trace_results[trace_id] = result_text

                        if (
                            status == EpisodeStatus.PLANNED
                            and _planned_counts_as_success(agent_name, spec)
                        ):
                            if agent_name == AgentName.BENCHMARK_PLANNER:
                                episode = await _fetch_episode(client, episode_id)
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

                            if spec.mode == EvalMode.BENCHMARK:
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
                                        log.info("eval_planned")
                                        success = True
                                break

                        if status == EpisodeStatus.COMPLETED:
                            episode = await _fetch_episode(client, episode_id)
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
            agent_stats["total"] += 1
            if success:
                agent_stats["success"] += 1

                worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
                handler = METRIC_HANDLERS.get(agent_name)
                if handler:
                    await handler(worker, session_id, agent_stats)
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
            return


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
        "--task-id", type=str, default=None, help="Run only a specific task ID"
    )
    parser.add_argument(
        "--verbose", action="store_true", help="Print backend traces during polling"
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

    # Configure logging to directory (logs/evals)
    log_dir = ROOT / "logs" / "evals"
    archive_dir = ROOT / "logs" / "archives"

    # Archive previous logs if they exist
    if log_dir.exists() and any(log_dir.iterdir()):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        archive_target = archive_dir / f"eval_run_{ts}"
        archive_dir.mkdir(parents=True, exist_ok=True)
        try:
            shutil.move(str(log_dir), str(archive_target))
        except Exception:
            # Fallback if move fails (e.g. across filesystems)
            shutil.copytree(str(log_dir), str(archive_target))
            shutil.rmtree(str(log_dir))

    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "run_evals.log"
    readable_log_file = log_dir / "readable_agent_logs.log"

    # Symlink for run_evals.log (latest.log in logs/evals/)
    latest_log = log_dir / "latest.log"
    if latest_log.exists():
        with contextlib.suppress(Exception):
            latest_log.unlink()
    with contextlib.suppress(Exception):
        latest_log.symlink_to(log_file.name)

    if readable_log_file.exists():
        with contextlib.suppress(Exception):
            readable_log_file.unlink()
    readable_log_file.touch()

    os.environ["LOG_DIR"] = str(log_dir)
    os.environ["EXTRA_DEBUG_LOG"] = str(log_file)
    os.environ["LOG_LEVEL"] = args.log_level
    configure_logging("evals")

    global logger
    global READABLE_AGENT_LOG_FILE
    logger = get_logger(__name__)
    READABLE_AGENT_LOG_FILE = readable_log_file

    print(f"Logging to: {log_file} (and symlinked at {latest_log})")
    print(f"Readable agent logs: {readable_log_file}")
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
                ("evals/controller.log", "controller.log"),
                ("evals/controller_debug.log", "controller_debug.log"),
                ("evals/worker_light.log", "worker_light.log"),
                ("evals/worker_heavy.log", "worker_heavy.log"),
                ("evals/worker_light_debug.log", "worker_light_debug.log"),
                ("evals/worker_heavy_debug.log", "worker_heavy_debug.log"),
                ("evals/temporal_worker.log", "temporal_worker.log"),
                ("evals/temporal_worker_debug.log", "temporal_worker_debug.log"),
                ("evals/frontend.log", "frontend.log"),
                ("evals/run_evals.log", "run_evals.log"),
                ("evals/readable_agent_logs.log", "readable_agent_logs.log"),
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

    stats = {
        agent: {
            "total": 0,
            "success": 0,
            "electrical_validity_rate": 0.0,
            "wire_integrity_rate": 0.0,
            "power_efficiency_score": 0.0,
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
                    if args.task_id:
                        data = [item for item in data if item["id"] == args.task_id]
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
                await run_single_eval(item, agent, stats, verbose=args.verbose)

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

    finish_time = time.time()
    duration = finish_time - start_time
    print(f"Agent Evals Finished at: {time.ctime(finish_time)}")
    print(f"Total Duration: {duration:.2f}s")
    logger.info("eval_run_finished", duration_s=round(duration, 2))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Agent evals interrupted.")
