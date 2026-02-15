import argparse
import asyncio
import json
import os
import sys
from pathlib import Path
from typing import Any

# Adjust path to include project root
sys.path.append(str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()

import httpx
from shared.enums import EpisodeStatus
from controller.clients.worker import WorkerClient

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:18001")


async def run_single_eval(item: dict[str, Any], agent_name: str):
    """
    Runs a single evaluation task for a specific agent via the Controller API.
    """
    task_id = item["id"]
    task_description = item["task"]

    print(f"Running eval: {task_id} for agent: {agent_name}")

    async with httpx.AsyncClient(timeout=60.0) as client:
        # 1. Trigger generation via API
        if agent_name.startswith("benchmark"):
            url = f"{CONTROLLER_URL}/benchmark/generate"
            payload = {"prompt": task_description}

            resp = await client.post(url, json=payload)
            if resp.status_code >= 400:
                print(
                    f"Error: Failed to trigger eval for {task_id}: {resp.status_code} - {resp.text}"
                )
                return

            data = resp.json()
            if "session_id" in data:
                session_id = data["session_id"]
                status_endpoint = f"benchmark/{session_id}"
            else:
                # Generic agent run returns episode_id
                session_id = data["episode_id"]
                status_endpoint = f"episodes/{session_id}"

        # 2. Poll for completion
        status_url = f"{CONTROLLER_URL}/{status_endpoint}"
        max_attempts = 120  # 10 minutes at 5s interval
        attempt = 0

        while attempt < max_attempts:
            await asyncio.sleep(5)
            attempt += 1

            try:
                status_resp = await client.get(status_url)
                if status_resp.status_code == 200:
                    status_data = status_resp.json()
                    status = status_data.get("status")

                    if status == EpisodeStatus.COMPLETED:
                        print(f"  [{task_id}] PASSED: Session completed successfully")
                        break

                    if status == EpisodeStatus.FAILED:
                        print(f"  [{task_id}] FAILED: Session failed in controller")
                        break

                    if attempt % 6 == 0:  # Log every 30s
                        print(f"  [{task_id}] Still running (status: {status})...")
                else:
                    print(
                        f"  [{task_id}] Warning: Error checking status: {status_resp.status_code}"
                    )
            except Exception as e:
                print(f"  [{task_id}] Warning: Exception during status check: {e}")

        if attempt >= max_attempts:
            print(f"  [{task_id}] TIMEOUT: Session did not complete within 10 minutes")

        # 3. Post-run verification (White-box checks if needed, using WorkerClient)
        if task_id == "bp-011":
            print(f"Verifying final side-effects for {task_id}...")
            worker = WorkerClient(base_url=WORKER_URL, session_id=session_id)
            files = await worker.list_files(".")
            file_paths = [f.path.lstrip("/") for f in files]
            required = [
                "benchmark_structure.md",
                "benchmark_engineer_todo.md",
                "objectives.yaml",
            ]
            for r in required:
                if r in file_paths:
                    print(f"  PASSED: Found {r}")
                else:
                    print(f"  FAILED: Missing required file {r}")

            try:
                obj_content = await worker.read_file("objectives.yaml")
                if "goal_zone" in obj_content and "x_min" not in obj_content:
                    print("  PASSED: objectives.yaml correctly modified")
                else:
                    print("  FAILED: objectives.yaml not correctly modified")
            except Exception as e:
                print(f"  FAILED: Error reading objectives.yaml: {e}")


async def main():
    parser = argparse.ArgumentParser(description="Run Agent Evals")

    available_agents = [
        "benchmark_planner",
        "benchmark_coder",
        "benchmark_reviewer",
        "engineer_planner",
        "engineer_coder",
        "engineer_reviewer",
    ]

    parser.add_argument(
        "--agent",
        type=str,
        # choices=available_agents + ["all"], # Optional: restrict choices strictly
        default="all",
        help="Agent to evaluate",
    )
    parser.add_argument(
        "--limit", type=int, default=0, help="Limit number of eval items per agent"
    )
    parser.add_argument(
        "--task-id", type=str, default=None, help="Run only a specific task ID"
    )
    args = parser.parse_args()

    # Check worker health before proceeding
    print(f"Checking worker health at {WORKER_URL}...")
    temp_client = WorkerClient(base_url=WORKER_URL, session_id="healthcheck")
    try:
        health = await temp_client.get_health()
        if health.get("status") != "healthy":
            print(f"Error: Worker is unhealthy: {health}")
            sys.exit(1)
    except Exception as e:
        print(
            f"Error: Could not connect to worker at {WORKER_URL}.\n"
            "Is 'docker compose up' running?"
        )
        print(f"Details: {e}")
        sys.exit(1)
    print("Worker is reachable and healthy.\n")

    # Load datasets
    # New structure: evals/datasets/[agent_name].json
    evals_root = Path(__file__).parent / "datasets"
    datasets = {}

    agents_to_run = available_agents if args.agent == "all" else [args.agent]

    for agent in agents_to_run:
        json_path = evals_root / f"{agent}.json"
        if json_path.exists():
            with open(json_path) as f:
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
            tasks.append(run_single_eval(item, agent))

    if tasks:
        await asyncio.gather(*tasks)
    else:
        print("No tasks to run.")

    print("Evals finished.")


if __name__ == "__main__":
    asyncio.run(main())
