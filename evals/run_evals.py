import argparse
import asyncio
import json
import logging
import os
import shutil
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import httpx
from dotenv import load_dotenv
from pyrate_limiter import Duration, Limiter, Rate

# Ensure repository root is importable when script is executed as a file.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controller.clients.worker import WorkerClient  # noqa: E402
from shared.enums import EpisodeStatus  # noqa: E402
from shared.logging import configure_logging, get_logger  # noqa: E402
from shared.utils.evaluation import analyze_electronics_metrics  # noqa: E402

load_dotenv()
# Logging will be configured in main() to support redirection to logs/evals
logger = get_logger(__name__)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


@dataclass(frozen=True)
class AgentEvalSpec:
    """Runtime details for an eval agent type."""

    mode: str  # benchmark | agent | git
    request_agent_name: str | None = None
    required_trace_names: tuple[str, ...] = ()


AGENT_SPECS: dict[str, AgentEvalSpec] = {
    # Benchmark graph roles
    "benchmark_planner": AgentEvalSpec(
        mode="benchmark", required_trace_names=("benchmark_planner",)
    ),
    "benchmark_coder": AgentEvalSpec(
        mode="benchmark", required_trace_names=("benchmark_coder",)
    ),
    "benchmark_reviewer": AgentEvalSpec(
        mode="benchmark", required_trace_names=("benchmark_reviewer",)
    ),
    # Mechanical engineering roles
    "engineer_planner": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_planner",
        required_trace_names=("planner",),
    ),
    "engineer_coder": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_coder",
        required_trace_names=("coder",),
    ),
    "engineer_reviewer": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_coder",
        required_trace_names=("execution_reviewer",),
    ),
    # Electrical engineering roles inside the unified engineer graph
    "electronics_planner": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_planner",
        required_trace_names=("electronics_planner",),
    ),
    "electronics_engineer": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_coder",
        required_trace_names=("electronics_engineer",),
    ),
    "electronics_reviewer": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_coder",
        required_trace_names=("electronics_reviewer",),
    ),
    # Sidecars
    "skill_agent": AgentEvalSpec(
        mode="agent",
        request_agent_name="engineer_coder",
        required_trace_names=("skill_learner",),
    ),
    "git_agent": AgentEvalSpec(mode="git"),
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
METRIC_HANDLERS = {
    "electronics_engineer": _handle_electronics_metrics,
}


def _episode_terminal(status: str | None) -> bool:
    if not status:
        return False
    return status in {
        EpisodeStatus.COMPLETED,
        EpisodeStatus.FAILED,
        EpisodeStatus.CANCELLED,
    }


async def _fetch_episode(client: httpx.AsyncClient, episode_id: str) -> dict[str, Any]:
    response = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
    response.raise_for_status()
    return response.json()


def _trace_names_lower(episode: dict[str, Any]) -> set[str]:
    traces = episode.get("traces") or []
    names = set()
    for trace in traces:
        name = trace.get("name")
        if isinstance(name, str) and name:
            names.add(name.lower())
    return names


def _missing_required_traces(
    required: tuple[str, ...], episode: dict[str, Any]
) -> list[str]:
    names = _trace_names_lower(episode)
    return [trace for trace in required if trace.lower() not in names]


async def _run_git_eval(item: dict[str, Any], stats: dict[str, Any], agent_name: str):
    task_id = item["id"]
    log = logger.bind(task_id=task_id, agent_name=agent_name, eval_mode="git")
    log.info("eval_start")

    session_id = f"eval-git-{task_id}-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)

    success = False
    failure_reason = ""

    try:
        await worker.git_init()
        await worker.write_file(
            "git_eval_note.md",
            f"# Git Eval {task_id}\n\n{item.get('task', '').strip()}\n",
            overwrite=True,
        )
        commit = await worker.git_commit(message=f"eval({task_id}): git agent baseline")
        status = await worker.git_status()

        if not commit.success:
            failure_reason = "git commit endpoint returned success=false"
        elif commit.commit_hash is None:
            failure_reason = "git commit produced no commit hash"
        elif getattr(status, "error", None):
            failure_reason = f"git status returned error: {status.error}"
        else:
            success = True
            log.info("eval_completed", session_id=session_id)
    except Exception as exc:
        failure_reason = str(exc)

    if not success:
        log.error("eval_failed", reason=failure_reason, session_id=session_id)

    agent_stats = stats[agent_name]
    agent_stats["total"] += 1
    if success:
        agent_stats["success"] += 1


async def run_single_eval(
    item: dict[str, Any], agent_name: str, stats: dict[str, Any], verbose: bool = False
):
    """
    Runs a single evaluation task for a specific agent type.
    """
    spec = AGENT_SPECS[agent_name]

    if spec.mode == "git":
        await _run_git_eval(item, stats, agent_name)
        return

    task_id = item["id"]
    task_description = item["task"]

    log = logger.bind(
        task_id=task_id,
        agent_name=agent_name,
        eval_mode=spec.mode,
    )
    log.info("eval_start")

    success = False
    session_id = ""

    async with httpx.AsyncClient(timeout=60.0) as client:
        if spec.mode == "benchmark":
            url = f"{CONTROLLER_URL}/benchmark/generate"
            payload = {"prompt": task_description}
            status_url_template = f"{CONTROLLER_URL}/benchmark/{{session_id}}"
            episode_id_key = "episode_id"
            session_id_key = "session_id"
        else:
            if agent_name == "electronics_engineer":
                session_id = f"EVAL-EE-{uuid.uuid4().hex[:8]}"
            else:
                session_id = f"eval-{task_id}-{uuid.uuid4().hex[:8]}"

            # Force electronics-engineer path to execute in integration-mode runs by
            # seeding explicit electronics requirements in objectives.yaml.
            if agent_name == "electronics_engineer":
                worker_for_seed = WorkerClient(
                    base_url=WORKER_LIGHT_URL, session_id=session_id
                )
                seeded_objectives = """objectives:
  goal_zone:
    min: [0, 0, 0]
    max: [10, 10, 10]
  build_zone:
    min: [-20, -20, 0]
    max: [20, 20, 40]
simulation_bounds:
  min: [-50, -50, -10]
  max: [50, 50, 80]
moved_object:
  label: "test_ball"
  shape: "sphere"
  start_position: [0, 0, 5]
  runtime_jitter: [0, 0, 0]
constraints:
  max_unit_cost: 50.0
  max_weight_g: 500.0
electronics_requirements:
  power_supply_available: true
"""
                await worker_for_seed.write_file(
                    "objectives.yaml", seeded_objectives, overwrite=True
                )

            url = f"{CONTROLLER_URL}/agent/run"
            payload = {
                "task": task_description,
                "agent_name": spec.request_agent_name or agent_name,
                "session_id": session_id,
            }
            status_url_template = f"{CONTROLLER_URL}/episodes/{{episode_id}}"
            episode_id_key = "episode_id"
            session_id_key = "session_id"

        try:
            resp = await client.post(url, json=payload)
            if resp.status_code >= 400:
                log.error(
                    "eval_trigger_failed",
                    status_code=resp.status_code,
                    response_text=resp.text,
                )
                stats[agent_name]["total"] += 1
                return
            data = resp.json()
            episode_id = str(data.get(episode_id_key) or data.get("episode_id"))
            session_id = str(
                data.get(session_id_key) or data.get("session_id") or episode_id
            )
        except Exception:
            log.exception("controller_request_failed")
            stats[agent_name]["total"] += 1
            return

        status_url = status_url_template.format(
            session_id=session_id, episode_id=episode_id
        )
        max_attempts = 120
        attempt = 0
        seen_trace_ids = set()

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
                            seen_trace_ids.add(trace_id)

                    if status == EpisodeStatus.PLANNED and spec.mode == "benchmark":
                        if agent_name == "benchmark_planner":
                            episode = await _fetch_episode(client, episode_id)
                            missing_traces = _missing_required_traces(
                                spec.required_trace_names, episode
                            )
                            if missing_traces:
                                log.error(
                                    "eval_failed_missing_traces",
                                    missing_traces=missing_traces,
                                )
                            else:
                                log.info("eval_planned")
                                success = True
                            break

                        log.info("eval_planned_confirming")
                        confirm_url = f"{CONTROLLER_URL}/benchmark/{session_id}/confirm"
                        await client.post(confirm_url)

                    if status == EpisodeStatus.COMPLETED:
                        episode = await _fetch_episode(client, episode_id)
                        missing_traces = _missing_required_traces(
                            spec.required_trace_names, episode
                        )
                        if missing_traces:
                            log.error(
                                "eval_failed_missing_traces",
                                missing_traces=missing_traces,
                            )
                        else:
                            log.info("eval_completed")
                            success = True
                        break

                    if status == EpisodeStatus.FAILED:
                        log.error("eval_failed")
                        break

                    if status == EpisodeStatus.CANCELLED:
                        log.warning("eval_cancelled")
                        break

                    if attempt % 6 == 0:
                        log.info("eval_still_running", status=status, attempt=attempt)
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
        default=1,
        help=(
            "Max number of eval tasks to run concurrently "
            "(default: 1 for real-LLM stability)"
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

    # Symlink for run_evals.log (latest.log in logs/evals/)
    latest_log = log_dir / "latest.log"
    if latest_log.exists():
        try:
            latest_log.unlink()
        except Exception:
            pass
    try:
        latest_log.symlink_to(log_file.name)
    except Exception:
        pass

    os.environ["LOG_DIR"] = str(log_dir)
    os.environ["EXTRA_DEBUG_LOG"] = str(log_file)
    os.environ["LOG_LEVEL"] = args.log_level
    configure_logging("evals")

    global logger
    logger = get_logger(__name__)

    print(f"Logging to: {log_file} (and symlinked at {latest_log})")
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
                env={**os.environ, "LOG_DIR": str(env_up_log_dir)},
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
                ("evals/worker_light.log", "worker_light.log"),
                ("evals/worker_heavy.log", "worker_heavy.log"),
                ("evals/temporal_worker.log", "temporal_worker.log"),
                ("evals/frontend.log", "frontend.log"),
                ("evals/run_evals.log", "run_evals.log"),
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
            logger.error("env_up_failed", stdout=e.stdout, stderr=e.stderr)
            print(f"ERROR: env_up.sh failed with exit code {e.returncode}")
            print(f"Check logs for details: {log_file}")
            sys.exit(1)
        except Exception:
            logger.exception("env_up_exception")
            sys.exit(1)

    # Rate limiter setup (50 RPM)
    rate = Rate(50, Duration.MINUTE)
    limiter = Limiter(rate)

    if args.agent != "all" and args.agent not in AGENT_SPECS:
        logger.error("unknown_agent", agent=args.agent)
        logger.info("available_agents")
        for name in available_agents:
            logger.info("agent", name=name)
        sys.exit(1)

    # Check health
    logger.info("controller_health_check_start", url=CONTROLLER_URL)
    async with httpx.AsyncClient(timeout=5.0) as client:
        try:
            health_resp = await client.get(f"{CONTROLLER_URL}/health")
            if (
                health_resp.status_code != 200
                or health_resp.json().get("status") != "healthy"
            ):
                logger.error("controller_unhealthy", response_text=health_resp.text)
                sys.exit(1)
        except Exception:
            logger.exception("controller_unreachable", url=CONTROLLER_URL)
            sys.exit(1)

    logger.info("worker_health_check_start", url=WORKER_LIGHT_URL)
    temp_client = WorkerClient(base_url=WORKER_LIGHT_URL, session_id="healthcheck")
    try:
        health = await temp_client.get_health()
        if health.get("status") != "healthy":
            logger.error("worker_unhealthy", health=health)
            sys.exit(1)
    except Exception:
        logger.exception("worker_unreachable", url=WORKER_LIGHT_URL)
        sys.exit(1)

    evals_root = Path(__file__).parent / "datasets"
    datasets = {}
    agents_to_run = available_agents if args.agent == "all" else [args.agent]

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
        json_path = evals_root / f"{agent}.json"
        if json_path.exists():
            with json_path.open() as f:
                try:
                    data = json.load(f)
                    if args.limit > 0:
                        data = data[: args.limit]
                    if args.task_id:
                        data = [item for item in data if item["id"] == args.task_id]
                    datasets[agent] = data
                except json.JSONDecodeError:
                    logger.error("dataset_json_decode_failed", path=str(json_path))
        else:
            logger.warning("dataset_missing", agent=agent, path=str(json_path))

    tasks = []
    for agent, dataset in datasets.items():
        logger.info("agent_evals_start", agent=agent, count=len(dataset))
        for item in dataset:
            tasks.append((item, agent))

    if tasks:
        semaphore = asyncio.Semaphore(max(1, args.concurrency))

        async def _guarded(item: dict[str, Any], agent: str):
            async with semaphore:
                if not args.no_rate_limit:
                    await asyncio.to_thread(limiter.delay_or_raise, "eval")
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
    asyncio.run(main())
