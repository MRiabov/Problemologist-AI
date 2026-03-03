import argparse
import asyncio
import json
import os
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import httpx
from dotenv import load_dotenv

# Ensure repository root is importable when script is executed as a file.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from controller.clients.worker import WorkerClient
from shared.enums import EpisodeStatus
from shared.utils.evaluation import analyze_electronics_metrics

load_dotenv()

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
    print(f"Running eval: {task_id} for agent: {agent_name}")

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
            print(f"  [{task_id}] COMPLETED")
    except Exception as exc:
        failure_reason = str(exc)

    if not success:
        print(f"  [{task_id}] FAILED: {failure_reason}")

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

    print(f"Running eval: {task_id} for agent: {agent_name}")

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
                print(
                    f"Error: Failed to trigger eval for {task_id}: "
                    f"{resp.status_code} - {resp.text}"
                )
                stats[agent_name]["total"] += 1
                return
            data = resp.json()
            episode_id = str(data.get(episode_id_key) or data.get("episode_id"))
            session_id = str(
                data.get(session_id_key) or data.get("session_id") or episode_id
            )
        except Exception as e:
            print(f"Error connecting to controller: {e}")
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

                    if verbose:
                        traces = status_data.get("traces", [])
                        for trace in sorted(traces, key=lambda t: t.get("id", 0)):
                            trace_id = trace.get("id")
                            if trace_id not in seen_trace_ids:
                                print(f"    [{task_id}] LOG: {trace.get('content')}")
                                seen_trace_ids.add(trace_id)

                    if status == EpisodeStatus.PLANNED and spec.mode == "benchmark":
                        if agent_name == "benchmark_planner":
                            episode = await _fetch_episode(client, episode_id)
                            missing_traces = _missing_required_traces(
                                spec.required_trace_names, episode
                            )
                            if missing_traces:
                                print(
                                    f"  [{task_id}] FAILED - missing traces: {', '.join(missing_traces)}"
                                )
                            else:
                                print(f"  [{task_id}] PLANNED")
                                success = True
                            break

                        print(f"  [{task_id}] PLANNED. Confirming...")
                        confirm_url = f"{CONTROLLER_URL}/benchmark/{session_id}/confirm"
                        await client.post(confirm_url)

                    if status == EpisodeStatus.COMPLETED:
                        episode = await _fetch_episode(client, episode_id)
                        missing_traces = _missing_required_traces(
                            spec.required_trace_names, episode
                        )
                        if missing_traces:
                            print(
                                f"  [{task_id}] FAILED - missing traces: {', '.join(missing_traces)}"
                            )
                        else:
                            print(f"  [{task_id}] COMPLETED")
                            success = True
                        break

                    if status == EpisodeStatus.FAILED:
                        print(f"  [{task_id}] FAILED")
                        break

                    if status == EpisodeStatus.CANCELLED:
                        print(f"  [{task_id}] CANCELLED")
                        break

                    if attempt % 6 == 0:
                        print(f"  [{task_id}] Still running (status: {status})...")
                else:
                    print(
                        f"  [{task_id}] Warning: Error checking status: "
                        f"{status_resp.status_code}"
                    )
            except Exception as e:
                print(f"  [{task_id}] Warning: Exception during status check: {e}")

        if attempt >= max_attempts:
            print(f"  [{task_id}] TIMEOUT")

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
        help="Max number of eval tasks to run concurrently (default: 1 for real-LLM stability)",
    )
    args = parser.parse_args()

    if args.agent != "all" and args.agent not in AGENT_SPECS:
        print(f"Error: Unknown agent '{args.agent}'.")
        print("Available agents:")
        for name in available_agents:
            print(f"  - {name}")
        sys.exit(1)

    # Check health
    print(f"Checking controller health at {CONTROLLER_URL}...")
    async with httpx.AsyncClient(timeout=5.0) as client:
        try:
            health_resp = await client.get(f"{CONTROLLER_URL}/health")
            if (
                health_resp.status_code != 200
                or health_resp.json().get("status") != "healthy"
            ):
                print(f"Error: Controller is unhealthy: {health_resp.text}")
                sys.exit(1)
        except Exception as e:
            print(f"Error: Could not connect to controller at {CONTROLLER_URL}: {e}")
            sys.exit(1)

    print(f"Checking worker health at {WORKER_LIGHT_URL}...")
    temp_client = WorkerClient(base_url=WORKER_LIGHT_URL, session_id="healthcheck")
    try:
        health = await temp_client.get_health()
        if health.get("status") != "healthy":
            print(f"Error: Worker is unhealthy: {health}")
            sys.exit(1)
    except Exception as e:
        print(f"Error: Could not connect to worker at {WORKER_LIGHT_URL}: {e}")
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
                    print(f"Error: Could not decode JSON at {json_path}")
        else:
            print(f"Warning: No dataset found for {agent} at {json_path}")

    tasks = []
    for agent, dataset in datasets.items():
        print(f"Starting evals for {agent}. Count: {len(dataset)}")
        for item in dataset:
            tasks.append((item, agent))

    if tasks:
        semaphore = asyncio.Semaphore(max(1, args.concurrency))

        async def _guarded(item: dict[str, Any], agent: str):
            async with semaphore:
                await run_single_eval(item, agent, stats, verbose=args.verbose)

        await asyncio.gather(*(_guarded(item, agent) for item, agent in tasks))
    else:
        print("No tasks to run.")

    print("\n" + "=" * 40)
    print("ARCHITECTURE EVALUATION REPORT")
    print("=" * 40)

    total_pass = 0
    total_count = 0
    for agent, s in stats.items():
        if s["total"] == 0:
            continue

        success_rate = (s["success"] / s["total"]) * 100
        total_pass += s["success"]
        total_count += s["total"]

        print(f"\nAgent: {agent}")
        print(f"  Task Success Rate: {success_rate:.1f}% ({s['success']}/{s['total']})")

        if agent in METRIC_HANDLERS and s["success"] > 0:
            print(
                "  Electrical Validity Rate: "
                f"{(s['electrical_validity_rate'] / s['success']) * 100:.1f}%"
            )
            print(
                "  Wire Integrity Rate: "
                f"{(s['wire_integrity_rate'] / s['success']) * 100:.1f}%"
            )
            print(
                "  Avg. Power Efficiency Score: "
                f"{(s['power_efficiency_score'] / s['success']):.2f}"
            )

    overall = (total_pass / total_count * 100) if total_count else 0.0
    print("\n" + "-" * 40)
    print(f"Overall pass rate: {overall:.1f}% ({total_pass}/{total_count})")


if __name__ == "__main__":
    asyncio.run(main())
