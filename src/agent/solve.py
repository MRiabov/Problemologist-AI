import argparse
import asyncio
import uuid

from rich.console import Console
from rich.panel import Panel

from src.agent.runner import run_agent
from src.agent.tools.env_adapter import register_runtime
from src.environment.persistence import DatabaseManager
from src.environment.runtime import ToolRuntime
from src.utils.paths import get_workspace_dir

console = Console()


async def solve_environment(problem_id: str, thread_id: str | None = None):
    """
    Initializes a ToolRuntime and has the agent solve it autonomously.
    """
    if thread_id is None:
        thread_id = f"solve_{str(uuid.uuid4())[:8]}"

    # 1. Initialize the Environment (ToolRuntime)
    workspace = get_workspace_dir() / thread_id
    db = DatabaseManager()  # Default to history.db

    runtime = ToolRuntime(workspace_dir=workspace, db=db, problem_id=problem_id)

    # 2. Register runtime for tools
    register_runtime(thread_id, runtime)

    # 3. Start session (which initializes the episode)
    runtime.start_session(thread_id)

    # Task description is usually passed from outside or loaded from problem
    # For now, we use a placeholder or load from problem_id if supported
    task_description = f"Solve CAD problem: {problem_id}"

    console.print(
        Panel(
            f"[bold blue]Autonomous Solve Sequence[/bold blue]\n"
            f"Problem ID: [green]{problem_id}[/green]\n"
            f"Thread ID: [yellow]{thread_id}[/yellow]\n"
            f"Task: {task_description}",
            title="Environment Solver",
        )
    )

    # 3. Hand over to the Agent
    # We pass the task_description as the starting query.
    # The agent will use its tools to interact with the environment (via files and sim bridge).
    try:
        await run_agent(task_description, thread_id=thread_id, runtime_id=thread_id)
        console.print(
            Panel(
                "[bold green]Agent solving sequence finished.[/bold green]",
                title="Environment Solver",
            )
        )
    except Exception as e:
        console.print(
            Panel(
                f"[bold red]Solver Failed:[/bold red] {e!s}",
                title="Environment Solver",
            )
        )
        raise e


async def main():
    parser = argparse.ArgumentParser(description="Autonomous Environment Solver")
    parser.add_argument(
        "--problem",
        help="The problem ID to solve (e.g., manifold_cube)",
        default="manifold_cube",
    )
    parser.add_argument(
        "--thread-id", help="Thread ID for continuing a session", default=None
    )

    args = parser.parse_args()

    try:
        await solve_environment(args.problem, args.thread_id)
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Solver interrupted by user.[/bold yellow]")
    except Exception as e:
        console.print(f"\n[bold red]Error in solver: {e!s}[/bold red]")


if __name__ == "__main__":
    asyncio.run(main())
