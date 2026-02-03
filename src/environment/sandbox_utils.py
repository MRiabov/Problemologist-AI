import json
import logging
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def run_sandboxed_script(
    sandbox: Any,
    script_content: str,
    result_file_name: str = "result.json",
    runner_file_name: str = "runner.py",
    timeout: int = 60,
    mount_src: bool = True,
    extra_mounts: Optional[List[Tuple[str, str]]] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Helper function to run a script in the sandbox and retrieve the result from a JSON file.

    Args:
        sandbox: The PodmanSandbox instance.
        script_content: The content of the Python script to run.
        result_file_name: The name of the JSON file where results will be written (relative to workspace).
        runner_file_name: The name of the Python runner script file to create.
        timeout: Execution timeout in seconds.
        mount_src: Whether to mount the src directory.
        extra_mounts: Additional mounts as list of (host_path, container_path) tuples.
        session_id: Optional session ID to run in a persistent container.

    Returns:
        A dictionary containing the parsed JSON result, or an error dict if something failed.
    """
    workspace_path = Path(sandbox.workspace_dir)
    runner_path = workspace_path / runner_file_name
    result_path = workspace_path / result_file_name

    try:
        # Write the runner script
        runner_path.write_text(script_content, encoding="utf-8")

        # Clean up previous result file
        result_path.unlink(missing_ok=True)

        # Execute script
        stdout, stderr, rc = sandbox.run_script(
            runner_file_name,
            timeout=timeout,
            mount_src=mount_src,
            extra_mounts=extra_mounts,
            session_id=session_id,
        )

        # Cleanup runner script
        if runner_path.exists():
            runner_path.unlink()

        # Check for timeouts or crashes
        if rc != 0:
            error_type = "TimeoutError" if rc == 124 else "CrashError"
            return {
                "status": "error",
                "error_type": error_type,
                "message": f"Sandbox execution failed (code {rc}): {stderr}",
                "stdout": stdout,
                "stderr": stderr,
            }

        # Read result file
        if result_path.exists():
            try:
                result_data = json.loads(result_path.read_text(encoding="utf-8"))
                # Cleanup result file
                result_path.unlink()
                return result_data
            except json.JSONDecodeError:
                return {
                    "status": "error",
                    "error_type": "JSONDecodeError",
                    "message": "Failed to decode result JSON.",
                    "stdout": stdout,
                    "stderr": stderr,
                }
        else:
            # Fallback: check if result was printed to stdout (legacy support or fallback)
            # This is less robust but some existing logic might rely on it.
            # However, the goal is to standardize on file-based.
            # For now, we return an error if file is missing, unless we want to try parsing stdout.
            # Let's keep it strict for the unified utility.
            return {
                "status": "error",
                "error_type": "NoResultFile",
                "message": f"Result file {result_file_name} was not created.",
                "stdout": stdout,
                "stderr": stderr,
            }

    except Exception as e:
        logger.error(f"Error running sandboxed script: {e}")
        return {
            "status": "error",
            "error_type": "HostError",
            "message": str(e),
        }
