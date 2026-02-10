import asyncio
import argparse
import json
import sys
import uuid
from pathlib import Path
from typing import Any

# Adjust path to include project root
sys.path.append(str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv()

from langfuse import Langfuse

from controller.graph.agent import create_agent_graph
from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient

WORKER_URL = "http://localhost:8001"


from langfuse.langchain import CallbackHandler


async def run_single_eval(item: dict[str, Any], agent_name: str):
    """
    Runs a single evaluation task for a specific agent.
    """
    task_id = item["id"]
    task_description = item["task"]
    
    print(f"Running eval: {task_id} for agent: {agent_name}")

    session_id = str(uuid.uuid4())
    client = WorkerClient(base_url=WORKER_URL, session_id=session_id)
    backend = RemoteFilesystemBackend(client)

    try:
        # Generate a trace_id for Langfuse (optional, controller will generate one if not passed)
        trace_id = uuid.uuid4().hex
        
        agent, _ = create_agent_graph(
            backend, 
            agent_name=agent_name, 
            trace_id=trace_id
        )

        await agent.ainvoke(
            {"messages": [("user", task_description)], "session_id": session_id},
            config={"metadata": {"eval_task_id": task_id, "agent_name": agent_name}},
        )

    except Exception as e:
        print(f"Eval {task_id} failed: {e}")


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
            with open(json_path, "r") as f:
                try:
                    data = json.load(f)
                    if args.limit > 0:
                        data = data[: args.limit]
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
