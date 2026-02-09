import asyncio
import json
import uuid
import os
import argparse
from typing import List, Dict, Any
from pathlib import Path
import sys

# Adjust path to include project root
sys.path.append(str(Path(__file__).parent.parent))

from langfuse import Langfuse
from langfuse.decorators import observe, langfuse_context

from controller.graph.agent import create_agent_graph
from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.config.settings import settings

WORKER_URL = "http://localhost:8001"


async def run_single_eval(item: Dict[str, Any], langfuse: Langfuse, agent_name: str):
    """
    Runs a single evaluation task for a specific agent.
    """
    task_id = item["id"]
    task_description = item["task"]

    print(f"Running eval: {task_id} for agent: {agent_name}")

    trace = langfuse.trace(
        name=f"eval_{agent_name}_{task_id}",
        id=str(uuid.uuid4()),
        metadata={"task_id": task_id, "agent": agent_name, "dataset_version": "v1"},
        input=task_description,
    )

    handler = trace.get_langchain_handler()
    session_id = str(uuid.uuid4())
    client = WorkerClient(base_url=WORKER_URL, session_id=session_id)
    backend = RemoteFilesystemBackend(client)

    try:
        # TODO: Modify create_agent_graph to accept an agent type/name or create specific graphs
        if agent_name == "planner":
            # For now, using the default graph as placeholder.
            # In real implementation, we would instantiation the specific subagent.
            agent = create_agent_graph(backend)
        elif agent_name == "engineer":
            agent = create_agent_graph(backend)
        else:
            agent = create_agent_graph(backend)

        result = await agent.ainvoke(
            {"messages": [("user", task_description)], "session_id": session_id},
            config={"callbacks": [handler]},
        )

        output = result["messages"][-1].content
        trace.update(output=output)

        trace.score(
            name="execution_success", value=1.0, comment="Agent ran without exception"
        )

    except Exception as e:
        print(f"Eval {task_id} failed: {e}")
        trace.update(output=f"Error: {str(e)}")
        trace.score(name="execution_success", value=0.0, comment=f"Exception: {e}")
    finally:
        langfuse.flush()


async def main():
    parser = argparse.ArgumentParser(description="Run Agent Evals")
    parser.add_argument(
        "--agent",
        type=str,
        choices=["planner", "engineer", "coder", "all"],
        default="all",
        help="Agent to evaluate",
    )
    parser.add_argument(
        "--limit", type=int, default=0, help="Limit number of eval items per agent"
    )
    args = parser.parse_args()

    # Load datasets
    evals_root = Path(__file__).parent
    datasets = {}

    agents_to_run = (
        ["planner", "engineer", "coder"] if args.agent == "all" else [args.agent]
    )

    for agent in agents_to_run:
        csv_path = evals_root / agent / "dataset.json"
        if csv_path.exists():
            with open(csv_path, "r") as f:
                data = json.load(f)
                if args.limit > 0:
                    data = data[: args.limit]
                datasets[agent] = data
        else:
            print(f"Warning: No dataset found for {agent} at {csv_path}")

    from dotenv import load_dotenv

    load_dotenv()

    langfuse = Langfuse()

    if not langfuse.auth_check():
        print("Langfuse auth check failed.")
        return

    tasks = []
    for agent, dataset in datasets.items():
        print(f"Starting evals for {agent}. Count: {len(dataset)}")
        for item in dataset:
            tasks.append(run_single_eval(item, langfuse, agent))

    await asyncio.gather(*tasks)
    print("Evals finished.")


if __name__ == "__main__":
    asyncio.run(main())
