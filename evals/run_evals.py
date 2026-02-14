import argparse
import asyncio
import json
import sys
import uuid
from pathlib import Path
from typing import Any

# Adjust path to include project root
sys.path.append(str(Path(__file__).parent.parent))

from dotenv import load_dotenv

load_dotenv()


from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.graph.agent import create_agent_graph

WORKER_URL = "http://localhost:18001"


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

        # Test a very simple operation first
        print(f"Testing connectivity for {agent_name}...")
        await backend.awrite("test_connect.txt", "hello")
        print(f"Connectivity test passed for {agent_name}")

        # For benchmark agents, we might need to initialize some files (like objectives.yaml template)
        if agent_name.startswith("benchmark"):
            from worker.objectives_template import OBJECTIVES_YAML_TEMPLATE

            await backend.awrite("objectives.yaml", OBJECTIVES_YAML_TEMPLATE)

        agent, _ = create_agent_graph(
            agent_name=agent_name, trace_id=trace_id, session_id=session_id
        )

        # Build initial state based on agent type
        if agent_name.startswith("benchmark"):
            from controller.agent.benchmark.models import GenerationSession

            session = GenerationSession(
                session_id=uuid.UUID(session_id), prompt=task_description
            )
            initial_state = {
                "session": session,
                "current_script": "",
                "mjcf_content": "",
                "simulation_result": None,
                "review_feedback": None,
                "review_round": 0,
                "plan": None,
                "messages": [],
            }
        else:
            initial_state = {
                "messages": [
                    (
                        "user",
                        task_description,
                    )
                ],
                "session_id": session_id,
            }

        await agent.ainvoke(
            initial_state,
            config={"metadata": {"eval_task_id": task_id, "agent_name": agent_name}},
        )

        # Post-run verification
        if task_id == "bp-011":
            print(f"Verifying outputs for {task_id}...")
            files = await client.list_files(".")
            file_paths = [f.path.lstrip("/") for f in files]
            required = [
                "benchmark_structure.md",
                "benchmark_engineer_todo.md",
                "objectives.yaml",
            ]
            for r in required:
                if r not in file_paths:
                    print(f"  FAILED: Missing required file {r}")
                else:
                    print(f"  PASSED: Found {r}")

            # Check objectives.yaml content
            obj_content = await client.read_file("objectives.yaml")
            if "# [TEMPLATE]" in obj_content:
                print("  PASSED: Header preserved in objectives.yaml")
            else:
                print("  FAILED: Header missing in objectives.yaml")

            # Check if it was actually modified (not just the template)
            if "goal_zone" in obj_content and "x_min" not in obj_content:
                print("  PASSED: objectives.yaml appears modified with real values")
            else:
                print("  FAILED: objectives.yaml does not appear correctly modified")

    except Exception as e:
        print(f"Eval {task_id} failed: {e}")
        import traceback

        traceback.print_exc()


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
