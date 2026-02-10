import asyncio
import sys
import uuid
import typer
from typing import Optional

from .graph import graph
from .state import AgentState
from .config import settings

app = typer.Typer()


@app.command()
def run(
    task: str = typer.Argument(..., help="The task for the agent to perform."),
    thread_id: Optional[str] = typer.Option(
        None, help="Optional thread ID for checkpointing."
    ),
):
    """Run the Engineer Agent on a specific task."""
    if not thread_id:
        thread_id = str(uuid.uuid4())

    asyncio.run(_run_agent(task, thread_id))


async def _run_agent(task: str, thread_id: str):
    config = {"configurable": {"thread_id": thread_id}}

    # Smart resume logic
    snapshot = await graph.get_state(config)
    inputs = None

    if snapshot.values:
        # State exists
        turn_count = snapshot.values.get("turn_count", 0)

        # Check if execution ended (next is empty)
        if not snapshot.next:
            if turn_count >= settings.max_agent_turns:
                print(
                    f"🔄 Turn limit reached ({turn_count}). Resetting counter and resuming."
                )
                await graph.update_state(config, {"turn_count": 0}, as_node="critic")
                inputs = None
            else:
                print(
                    "⚠️ Plan already completed or terminated. Use a new thread_id to restart."
                )
                return
        else:
            # Suspended or interrupted
            print(f"▶️ Resuming existing session (Turn {turn_count})...")
            inputs = None
    else:
        # New session
        print(f"🆕 Starting new session...")
        inputs = AgentState(task=task)

    print(f"🚀 Starting Agent with thread_id: {thread_id}")
    print(f"📝 Task: {task}")
    print(
        f"🔧 Config: Worker={settings.spec_001_api_url}, Session={settings.default_session_id}"
    )
    print("---")

    async for event in graph.astream(inputs, config=config):
        for node_name, output in event.items():
            print(f"\n[Node: {node_name}]")

            if "status" in output and output["status"]:
                print(f"Status: {output['status']}")

            if "current_step" in output and output["current_step"]:
                print(f"Step: {output['current_step']}")

            if "feedback" in output and output["feedback"]:
                print(f"Feedback: {output['feedback']}")

            if "journal" in output:
                # Print only the latest entry if possible
                new_entries = output["journal"].split("\n")[-5:]
                print("Journal (latest):")
                for entry in new_entries:
                    if entry.strip():
                        print(f"  {entry.strip()}")

    print("\n---")
    print("✅ Run complete.")


if __name__ == "__main__":
    app()
