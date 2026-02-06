import asyncio
import sys
import uuid
import typer
from typing import Optional

from .graph import graph
from .state import AgentState

app = typer.Typer()

@app.command()
def run(
    task: str = typer.Argument(..., help="The task for the agent to perform."),
    thread_id: Optional[str] = typer.Option(None, help="Optional thread ID for checkpointing.")
):
    """Run the Engineer Agent on a specific task."""
    if not thread_id:
        thread_id = str(uuid.uuid4())
    
    asyncio.run(_run_agent(task, thread_id))

async def _run_agent(task: str, thread_id: str):
    config = {"configurable": {"thread_id": thread_id}}
    initial_state = AgentState(task=task)
    
    print(f"🚀 Starting Agent with thread_id: {thread_id}")
    print(f"📝 Task: {task}
")
    print("---")

    async for event in graph.astream(initial_state, config=config):
        for node_name, output in event.items():
            print(f"
[Node: {node_name}]")
            
            if "status" in output and output["status"]:
                print(f"Status: {output['status']}")
            
            if "current_step" in output and output["current_step"]:
                print(f"Step: {output['current_step']}")
            
            if "feedback" in output and output["feedback"]:
                print(f"Feedback: {output['feedback']}")
                
            if "journal" in output:
                # Print only the latest entry if possible
                new_entries = output["journal"].split("
")[-5:]
                print("Journal (latest):")
                for entry in new_entries:
                    if entry.strip():
                        print(f"  {entry.strip()}")

    print("
---")
    print("✅ Run complete.")

if __name__ == "__main__":
    app()
