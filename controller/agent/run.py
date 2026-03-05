import asyncio
import uuid

import typer

from .config import settings
from .execution_limits import resolve_agent_execution_policy
from .graph import graph
from .state import AgentState

app = typer.Typer()


@app.command()
def run(
    task: str = typer.Argument(..., help="The task for the agent to perform."),
    thread_id: str | None = typer.Option(
        None, help="Optional thread ID for checkpointing."
    ),
):
    """Run the Engineer Agent on a specific task."""
    if not thread_id:
        thread_id = str(uuid.uuid4())

    asyncio.run(_run_agent(task, thread_id))


async def _run_agent(task: str, thread_id: str):
    config = {"configurable": {"thread_id": thread_id}}
    turn_limit = resolve_agent_execution_policy("engineer_coder").max_turns

    # Smart resume logic
    snapshot = await graph.get_state(config)
    inputs = None

    if snapshot.values:
        # State exists
        turn_count = snapshot.values.get("turn_count", 0)
        existing_episode_id = snapshot.values.get("episode_id")
        if not existing_episode_id:
            generated_episode_id = str(uuid.uuid4())
            await graph.update_state(
                config, {"episode_id": generated_episode_id}, as_node="engineer_planner"
            )
            print(f"Missing episode_id in checkpoint; assigned {generated_episode_id}.")

        # Check if execution ended (next is empty)
        if not snapshot.next:
            if turn_count >= turn_limit:
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
        print("🆕 Starting new session...")
        inputs = AgentState(task=task, session_id=thread_id)

    print(f"🚀 Starting Agent with thread_id: {thread_id}")
    print(f"📝 Task: {task}")
    print(
        f"🔧 Config: Worker={settings.worker_light_url}, Session={settings.default_session_id}"
    )
    print("---")

    async for event in graph.astream(inputs, config=config):
        for node_name, output in event.items():
            print(f"\n[Node: {node_name}]")

            if output.get("status"):
                print(f"Status: {output['status']}")

            if output.get("current_step"):
                print(f"Step: {output['current_step']}")

            if output.get("feedback"):
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
