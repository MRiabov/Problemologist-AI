"""Python runtime executor for sandboxed code execution.

Provides secure execution of Python code with timeout enforcement
and subprocess isolation.
"""

import asyncio
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path

import structlog
from pydantic import BaseModel

logger = structlog.get_logger(__name__)

# Default execution timeout in seconds
DEFAULT_TIMEOUT_SECONDS = 30


class ExecutionResult(BaseModel):
    """Result of a Python code execution."""

    stdout: str
    stderr: str
    exit_code: int
    timed_out: bool = False


@dataclass
class RuntimeConfig:
    """Configuration for the Python runtime executor."""

    timeout_seconds: int = DEFAULT_TIMEOUT_SECONDS
    python_executable: str = "python3"
    working_directory: str | None = None


def run_python_code(
    code: str,
    env: dict[str, str] | None = None,
    config: RuntimeConfig | None = None,
) -> ExecutionResult:
    """Execute Python code in a subprocess.

    Args:
        code: Python code to execute.
        env: Optional environment variables for the subprocess.
        config: Optional runtime configuration.

    Returns:
        ExecutionResult containing stdout, stderr, exit_code, and timeout status.
    """
    if config is None:
        config = RuntimeConfig()

    logger.debug(
        "runtime_execute",
        code_length=len(code),
        timeout=config.timeout_seconds,
    )

    # Set up PYTHONPATH to include src/worker for utils import
    actual_env = env.copy() if env else {}
    current_pythonpath = actual_env.get("PYTHONPATH", "")
    
    # Get absolute path to src/worker
    project_root = Path(__file__).parent.parent.parent.parent
    worker_path = str(project_root / "src" / "worker")
    
    if current_pythonpath:
        actual_env["PYTHONPATH"] = f"{worker_path}:{current_pythonpath}"
    else:
        actual_env["PYTHONPATH"] = worker_path

    # Write code to a temporary file
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".py",
        delete=False,
    ) as f:
        f.write(code)
        script_path = f.name

    try:
        # Build subprocess command
        cmd = [config.python_executable, script_path]

        # Run the subprocess
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=config.timeout_seconds,
            env=actual_env,
            cwd=config.working_directory,
        )

        logger.info(
            "runtime_execute_complete",
            exit_code=result.returncode,
            stdout_length=len(result.stdout),
            stderr_length=len(result.stderr),
        )

        return ExecutionResult(
            stdout=result.stdout,
            stderr=result.stderr,
            exit_code=result.returncode,
            timed_out=False,
        )

    except subprocess.TimeoutExpired:
        logger.warning("runtime_execute_timeout", timeout=config.timeout_seconds)
        return ExecutionResult(
            stdout="",
            stderr=f"Execution timed out after {config.timeout_seconds} seconds",
            exit_code=-1,
            timed_out=True,
        )

    except Exception as e:
        logger.error("runtime_execute_error", error=str(e))
        return ExecutionResult(
            stdout="",
            stderr=str(e),
            exit_code=-1,
            timed_out=False,
        )

    finally:
        # Clean up temporary file
        Path(script_path).unlink(missing_ok=True)


async def run_python_code_async(
    code: str,
    env: dict[str, str] | None = None,
    config: RuntimeConfig | None = None,
) -> ExecutionResult:
    """Execute Python code asynchronously in a subprocess.

    Args:
        code: Python code to execute.
        env: Optional environment variables for the subprocess.
        config: Optional runtime configuration.

    Returns:
        ExecutionResult containing stdout, stderr, exit_code, and timeout status.
    """
    if config is None:
        config = RuntimeConfig()

    logger.debug(
        "runtime_execute_async",
        code_length=len(code),
        timeout=config.timeout_seconds,
    )

    # Set up PYTHONPATH to include src/worker for utils import
    actual_env = env.copy() if env else {}
    current_pythonpath = actual_env.get("PYTHONPATH", "")
    
    # Get absolute path to src/worker
    project_root = Path(__file__).parent.parent.parent.parent
    worker_path = str(project_root / "src" / "worker")
    
    if current_pythonpath:
        actual_env["PYTHONPATH"] = f"{worker_path}:{current_pythonpath}"
    else:
        actual_env["PYTHONPATH"] = worker_path

    # Write code to a temporary file
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".py",
        delete=False,
    ) as f:
        f.write(code)
        script_path = f.name

    try:
        # Create subprocess
        process = await asyncio.create_subprocess_exec(
            config.python_executable,
            script_path,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            env=actual_env,
            cwd=config.working_directory,
        )

        try:
            # Wait for completion with timeout
            stdout, stderr = await asyncio.wait_for(
                process.communicate(),
                timeout=config.timeout_seconds,
            )

            exit_code = process.returncode or 0

            logger.info(
                "runtime_execute_async_complete",
                exit_code=exit_code,
                stdout_length=len(stdout),
                stderr_length=len(stderr),
            )

            return ExecutionResult(
                stdout=stdout.decode("utf-8"),
                stderr=stderr.decode("utf-8"),
                exit_code=exit_code,
                timed_out=False,
            )

        except TimeoutError:
            # Kill the process if it times out
            process.kill()
            await process.wait()

            logger.warning(
                "runtime_execute_async_timeout",
                timeout=config.timeout_seconds,
            )

            return ExecutionResult(
                stdout="",
                stderr=f"Execution timed out after {config.timeout_seconds} seconds",
                exit_code=-1,
                timed_out=True,
            )

    except Exception as e:
        logger.error("runtime_execute_async_error", error=str(e))
        return ExecutionResult(
            stdout="",
            stderr=str(e),
            exit_code=-1,
            timed_out=False,
        )

    finally:
        # Clean up temporary file
        Path(script_path).unlink(missing_ok=True)