import multiprocessing
import traceback
import time
import queue as queue_module
from typing import Any, Dict
from .simulation import SimulationLoop


class SimulationError(Exception):
    """Base class for simulation errors."""
    pass


class SimulationTimeoutError(SimulationError):
    """Raised when a simulation exceeds its allocated time."""
    pass


class SimulationCrashError(SimulationError):
    """Raised when the simulation subprocess crashes."""
    pass


def _run_sim_wrapper(model_path: str, agent_script: str, max_steps: int, queue: multiprocessing.Queue):
    """
    Internal wrapper run in a separate process.
    """
    try:
        sim = SimulationLoop(model_path)
        result = sim.run(agent_script, max_steps=max_steps)
        queue.put(result)
    except Exception:
        queue.put({
            "status": "ERROR",
            "message": f"Subprocess exception: {traceback.format_exc()}"
        })


def run_isolated(model_path: str, agent_script: str, max_steps: int = 1000, timeout: float = 30.0) -> Dict[str, Any]:
    """
    Runs a simulation in a separate process with a timeout.
    """
    queue = multiprocessing.Queue()
    process = multiprocessing.Process(
        target=_run_sim_wrapper,
        args=(model_path, agent_script, max_steps, queue)
    )

    process.start()
    
    start_time = time.time()
    result = None
    
    while True:
        try:
            # Short wait to allow checking process liveness
            result = queue.get(timeout=0.1)
            break
        except queue_module.Empty:
            # Check if process is still running
            if not process.is_alive():
                # One last check of the queue just in case it finished and died between checks
                try:
                    result = queue.get(timeout=0.01)
                except queue_module.Empty:
                    pass
                break
            
            # Check for overall timeout
            if time.time() - start_time > timeout:
                break
    
    if process.is_alive():
        process.terminate()
        process.join()

    if result is not None:
        return result
    
    if time.time() - start_time > timeout:
        return {
            "status": "TIMEOUT",
            "message": f"Simulation timed out after {timeout} seconds"
        }
    
    if process.exitcode != 0:
        return {
            "status": "CRASH",
            "message": f"Simulation process crashed with exit code {process.exitcode}"
        }
    
    return {
        "status": "ERROR",
        "message": "Simulation process exited unexpectedly without result"
    }