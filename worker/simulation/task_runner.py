import multiprocessing
import traceback
from pathlib import Path

# We need to import the things needed for simulation
from worker.utils import simulate
from worker.utils.loader import load_component_from_script


def run_simulation_task(
    script_path: str,
    script_content: str | None,
    session_root: Path,
    smoke_test_mode: bool,
    backend: str,
):
    try:
        # Load component
        # We need to resolve the script path relative to session root?
        # The caller usually resolves it. Let's assume script_path is absolute or resolvable.
        # But wait, session_root is a Path. Path might pickle okay?

        # Re-resolve if needed or trust the path.
        # Ideally we change directory to session_root?

        component = load_component_from_script(
            script_path=Path(script_path),
            session_root=session_root,
            script_content=script_content,
        )

        result = simulate(
            component,
            output_dir=session_root,
            smoke_test_mode=smoke_test_mode,
            backend=backend,
        )

        # Return serializable result
        # SimulationMetrics is a Pydantic model, likely picklable.
        return result
    except Exception as e:
        # Return exception info
        return {"error": str(e), "traceback": traceback.format_exc()}
