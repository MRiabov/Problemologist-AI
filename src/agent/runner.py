import argparse
import asyncio
import sys
import uuid
from pathlib import Path

# Add the project root to sys.path to allow importing from src
root_path = Path(__file__).resolve().parent.parent.parent
sys.path.append(str(root_path))

from langchain_core.messages import HumanMessage
from rich.panel import Panel

from src.agent.graph.graph import build_graph
from src.agent.utils.checkpoint import get_checkpointer
from src.agent.utils.visualize import console, visualize_event
from src.agent.utils.logging import setup_logging

async def run_agent(query: str, thread_id: str = None):
    """
    Initializes and runs the agent graph with the given query.
    """
    setup_logging()
    if thread_id is None:
        thread_id = str(uuid.uuid4())

    # 1. Initialize Checkpointer
    # get_checkpointer returns an AsyncSqliteSaver (which is an async context manager)
    async with get_checkpointer() as checkpointer:
        # 2. Build and Compile Graph
        # We use the checkpointer for persistence
        app = build_graph().compile(checkpointer=checkpointer)

        # 3. Prepare initial state and config
        initial_state = {
            "messages": [HumanMessage(content=query)],
            "plan": "",
            "step_count": 0,
            "scratchpad": {},
        }

        config = {"configurable": {"thread_id": thread_id}}

        console.print(
            Panel(
                f"[bold green]Starting Agent Session[/bold green]\nThread ID: {thread_id}",
                title="System",
            )
        )

        # 4. Run the graph asynchronously and visualize events
        async for event in app.astream(initial_state, config, stream_mode="updates"):
            visualize_event(event)

        console.print(
            Panel("[bold green]Agent Task Completed[/bold green]", title="System")
        )


async def main():
    parser = argparse.ArgumentParser(description="VLM CAD Agent Runner")
    parser.add_argument("query", help="The design request or question for the agent")
    parser.add_argument(
        "--thread-id", help="Thread ID for continuing a session", default=None
    )

    args = parser.parse_args()

    try:
        await run_agent(args.query, args.thread_id)
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Agent interrupted by user.[/bold yellow]")
    except Exception as e:
        console.print(f"\n[bold red]Error running agent: {e!s}[/bold red]")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
