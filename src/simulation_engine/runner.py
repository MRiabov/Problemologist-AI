import multiprocessing
import traceback
from typing import Any

from src.agent.utils.config import Config
from src.simulation_engine.bridge import MujocoBridge
from src.environment.sandbox import PodmanSandbox


def _run_sim_wrapper(
    xml_string: str,
    duration: float,
    queue: multiprocessing.Queue,
    agent_script: str = "",
    goal_pos: tuple[float, float, float] | None = None,
):
    """
    Internal wrapper to run the simulation and put the result in a queue.
    This runs in a separate process.
    """
    try:
        # Explicitly initialize dependencies for the isolated process
        workspace_dir = Config.WORKSPACE_DIR
        sandbox = PodmanSandbox(str(workspace_dir))

        bridge = MujocoBridge(workspace_dir=workspace_dir, sandbox=sandbox)
        # Use _run_simulation_internal (synchronous) instead of async run_simulation
        result = bridge._run_simulation_internal(
            xml_string, duration=duration, agent_script=agent_script, goal_pos=goal_pos
        )
        # SimResult is a Pydantic model, use model_dump() not asdict()
        queue.put({"success": True, "result": result.model_dump()})
    except Exception as e:
        error_type = "RuntimeError"
        message = str(e)
        if "CRASH_DETECTED" in message:
            error_type = "CrashError"
            # Extract exit code and stderr if possible
            parts = message.split(":", 2)
            if len(parts) >= 3:
                message = f"Sandbox crashed (code {parts[1]}): {parts[2]}"
            else:
                message = "Sandbox crashed."

        queue.put(
            {
                "success": False,
                "error_type": error_type,
                "message": message,
                "traceback": traceback.format_exc(),
            }
        )


def run_isolated(
    xml_string: str,
    duration: float = 5.0,
    timeout: float = 30.0,
    agent_script: str = "",
    goal_pos: tuple[float, float, float] | None = None,
) -> dict[str, Any]:
    """
    Runs the simulation in an isolated process with a timeout.

    Args:
        xml_string: The MJCF XML content.
        duration: The simulation duration in seconds.
        timeout: Maximum wall-clock time allowed for the simulation process.
        agent_script: Python control logic.
        goal_pos: Optional success zone coordinates.

    Returns:
        A dictionary containing the simulation results or error information.
    """
    queue = multiprocessing.Queue()
    process = multiprocessing.Process(
        target=_run_sim_wrapper,
        args=(xml_string, duration, queue, agent_script, goal_pos),
    )

    # Use a process group to ensure we can kill all sub-processes
    # This is tricky with multiprocessing.Process, so we'll rely on it for now
    # but the REAL issue is often child processes of the target.

    process.start()
    process.join(timeout=timeout)

    if process.is_alive():
        # On Linux, terminate sends SIGTERM to the process.
        # To be more aggressive and handle potential orphans, we might want
        # to use something like psutil to kill the whole tree, or have the
        # child process manage its own children better.
        process.terminate()
        process.join(timeout=2)
        if process.is_alive():
            process.kill()
            process.join()

        return {
            "success": False,
            "error_type": "TimeoutError",
            "message": f"Simulation timed out after {timeout} seconds.",
        }

    if process.exitcode != 0:
        return {
            "success": False,
            "error_type": "CrashError",
            "message": f"Simulation process crashed with exit code {process.exitcode}.",
        }

    if queue.empty():
        return {
            "success": False,
            "error_type": "UnknownError",
            "message": "Simulation process finished but returned no results.",
        }

    return queue.get()
