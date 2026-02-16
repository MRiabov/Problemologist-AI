import argparse
import asyncio
import json
import os
import sys
import uuid
from pathlib import Path
from typing import Any

import httpx
from dotenv import load_dotenv

from controller.clients.worker import WorkerClient
from shared.enums import EpisodeStatus
from shared.utils.evaluation import analyze_electronics_metrics

load_dotenv()

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:18001")


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


async def run_single_eval(
    item: dict[str, Any], agent_name: str, stats: dict[str, Any], verbose: bool = False
):
    """
    Runs a single evaluation task for a specific agent via the Controller API.
    """
    task_id = item["id"]
    task_description = item["task"]

    print(f"Running eval: {task_id} for agent: {agent_name}")

    async with httpx.AsyncClient(timeout=60.0) as client:
        # 1. Trigger generation/run via API
        if agent_name.startswith("benchmark"):
            url = f"{CONTROLLER_URL}/benchmark/generate"
            payload = {"prompt": task_description}
            status_prefix = "benchmark"
        else:
            url = f"{CONTROLLER_URL}/agent/run"
            payload = {
                "task": task_description,
                "agent_name": agent_name,
                "session_id": f"eval-{task_id}-{uuid.uuid4().hex[:8]}",
            }
            status_prefix = "episodes"

        try:
            resp = await client.post(url, json=payload)
            if resp.status_code >= 400:
                print(
                    f"Error: Failed to trigger eval for {task_id}: "
                    f"{resp.status_code} - {resp.text}"
                )
                return

            data = resp.json()
            session_id = data.get("session_id") or data.get("episode_id")
        except Exception as e:
            print(f"Error connecting to controller: {e}")
            return

        # 2. Polling loop
        status_url = f"{CONTROLLER_URL}/{status_prefix}/{session_id}"
        max_attempts = 120  # 10 minutes at 5s interval
        attempt = 0
        seen_trace_ids = set()
        success = False

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
                        for trace in sorted(traces, key=lambda t: t["id"]):
                            if trace["id"] not in seen_trace_ids:
                                print(f"    [{task_id}] LOG: {trace.get('content')}")
                                seen_trace_ids.add(trace["id"])

                    if status == EpisodeStatus.COMPLETED:
                        print(f"  [{task_id}] COMPLETED")
                        success = True
                        break

                    if status == EpisodeStatus.FAILED:
                        print(f"  [{task_id}] FAILED")
                        break

                    if status == EpisodeStatus.CANCELLED:
                        print(f"  [{task_id}] CANCELLED")
                        break

                    if status == EpisodeStatus.PLANNED and agent_name.startswith(
                        "benchmark"
                    ):
                        print(f"  [{task_id}] PLANNED. Confirming...")
                        confirm_url = f"{CONTROLLER_URL}/benchmark/{session_id}/confirm"
                        await client.post(confirm_url)

                    if attempt % 6 == 0:  # Every 30s
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

        # Update stats
        agent_stats = stats[agent_name]
        agent_stats["total"] += 1
        if success:
            agent_stats["success"] += 1

            # 3. Post-run verification & Metrics
            worker = WorkerClient(base_url=WORKER_URL, session_id=session_id)
            handler = METRIC_HANDLERS.get(agent_name)
            if handler:
                await handler(worker, session_id, agent_stats)


async def main():
    parser = argparse.ArgumentParser(description="Run Agent Evals")

    available_agents = [
        "benchmark_planner",
        "benchmark_coder",
        "benchmark_reviewer",
        "engineer_planner",
        "engineer_coder",
        "engineer_reviewer",
        "electronics_engineer",
    ]

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
    args = parser.parse_args()

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

    print(f"Checking worker health at {WORKER_URL}...")
    temp_client = WorkerClient(base_url=WORKER_URL, session_id="healthcheck")
    try:
        health = await temp_client.get_health()
        if health.get("status") != "healthy":
            print(f"Error: Worker is unhealthy: {health}")
            sys.exit(1)
    except Exception as e:
        print(f"Error: Could not connect to worker at {WORKER_URL}: {e}")

    # Load datasets
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
            tasks.append(run_single_eval(item, agent, stats, verbose=args.verbose))

    if tasks:
        await asyncio.gather(*tasks)
    else:
        print("No tasks to run.")

    # Generate Report
    print("\n" + "=" * 40)
    print("ELECTROMECHANICAL EVALUATION REPORT")
    print("=" * 40)

    for agent, s in stats.items():
        if s["total"] == 0:
            continue

        success_rate = (s["success"] / s["total"]) * 100
        print(f"\nAgent: {agent}")
        print(f"  Task Success Rate: {success_rate:.1f}% ({s['success']}/{s['total']})")

        if agent == "electronics_engineer" and s["success"] > 0:
            v_rate = (s["electrical_validity_rate"] / s["success"]) * 100
            i_rate = (s["wire_integrity_rate"] / s["success"]) * 100
            e_score = (s["power_efficiency_score"] / s["success"]) * 100
            print(f"  Electrical Validity Rate: {v_rate:.1f}%")
            print(f"  Wire Integrity Rate: {i_rate:.1f}%")
            print(f"  Power Efficiency Score: {e_score:.1f}%")

    print("\nEvals finished.")


if __name__ == "__main__":
    asyncio.run(main())
