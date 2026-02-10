import asyncio
import argparse
import json
import sys
import uuid
from pathlib import Path
from typing import Any

# Adjust path to include project root
sys.path.append(str(Path(__file__).parent.parent))

from langfuse import Langfuse

from controller.graph.agent import create_agent_graph
from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient

WORKER_URL = "http://localhost:8001"


async def run_single_eval(item: dict[str, Any], langfuse: Langfuse, agent_name: str):
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
        # Create agent logic is now centralized in create_agent_graph with mapping
        agent = create_agent_graph(backend, agent_name=agent_name)

        config = {}
        if handler:
            config["callbacks"] = [handler]

        result = await agent.ainvoke(
            {"messages": [("user", task_description)], "session_id": session_id},
            config=config,
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

    from dotenv import load_dotenv

    load_dotenv()

    import os

    use_langfuse = False
    langfuse = None

    if os.getenv("LANGFUSE_PUBLIC_KEY") and os.getenv("LANGFUSE_SECRET_KEY"):
        try:
            langfuse = Langfuse()
            if langfuse.auth_check():
                use_langfuse = True
            else:
                print(
                    "Langfuse auth check failed. Continuing execution without logging."
                )
        except Exception as e:
            print(
                f"Langfuse initialization failed: {e}. Continuing execution without logging."
            )
    else:
        print("Langfuse credentials not found. Continuing execution without logging.")

    # Dummy class for when Langfuse is disabled
    class DummyTrace:
        def get_langchain_handler(self):
            return None

        def update(self, **kwargs):
            pass

        def score(self, **kwargs):
            pass

    tasks = []
    for agent, dataset in datasets.items():
        print(f"Starting evals for {agent}. Count: {len(dataset)}")
        for item in dataset:
            # We need to wrap process so we can pass the right object
            real_trace_provider = langfuse if use_langfuse else None

            # If no langfuse, we'll patch run_single_eval to use dummy
            # Actually easiest is to just pass a dummy "langfuse" object that has .trace()

            if use_langfuse:
                tasks.append(run_single_eval(item, langfuse, agent))
            else:
                # Create a dummy provider that returns a DummyTrace
                class DummyProvider:
                    def trace(self, **kwargs):
                        return DummyTrace()

                    def flush(self):
                        pass

                tasks.append(run_single_eval(item, DummyProvider(), agent))

    if tasks:
        await asyncio.gather(*tasks)
    else:
        print("No tasks to run.")

    print("Evals finished.")


if __name__ == "__main__":
    asyncio.run(main())
