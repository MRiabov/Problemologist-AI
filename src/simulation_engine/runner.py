import multiprocessing
import traceback
from dataclasses import asdict
from typing import Any

from src.compiler.mujoco_bridge import MujocoBridge


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
        bridge = MujocoBridge()
        result = bridge.run_simulation(
            xml_string, duration=duration, agent_script=agent_script, goal_pos=goal_pos
        )
        queue.put({"success": True, "result": asdict(result)})
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

    process.start()
    process.join(timeout=timeout)

    if process.is_alive():
        process.terminate()
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
