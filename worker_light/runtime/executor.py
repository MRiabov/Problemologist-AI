"""Python runtime executor for sandboxed code execution.

Provides secure execution of Python code with timeout enforcement
and subprocess isolation.
"""

import asyncio
import subprocess
import tempfile
from pathlib import Path

import structlog
from pydantic import BaseModel, StrictBool, StrictInt, StrictStr

from shared.type_checking import type_check

logger = structlog.get_logger(__name__)

# Default execution timeout in seconds
DEFAULT_TIMEOUT_SECONDS = 30


class ExecutionResult(BaseModel):
    """Result of a Python code execution."""

    stdout: StrictStr
    stderr: StrictStr
    exit_code: StrictInt
    timed_out: StrictBool = False


class RuntimeConfig(BaseModel):
    """Configuration for the Python runtime executor."""

    timeout_seconds: StrictInt = DEFAULT_TIMEOUT_SECONDS
    python_executable: StrictStr = "python3"
    working_directory: StrictStr | None = None


from contextlib import contextmanager


def _prepare_execution(
    code: str, env: dict[str, str] | None
) -> tuple[str, dict[str, str]]:
    """Common logic for setting up PYTHONPATH and temporary file."""
    import os

    actual_env = os.environ.copy()
    if env:
        actual_env.update(env)

    current_pythonpath = actual_env.get("PYTHONPATH", "")
    project_root = Path(__file__).resolve().parents[2]
    worker_package_parent = str(project_root)

    if current_pythonpath:
        actual_env["PYTHONPATH"] = f"{worker_package_parent}:{current_pythonpath}"
    else:
        actual_env["PYTHONPATH"] = worker_package_parent

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".py",
        delete=False,
    ) as f:
        f.write(code)
        script_path = f.name

    return script_path, actual_env


@contextmanager
def _execution_context(code: str, env: dict[str, str] | None):
    """Context manager for execution preparation and cleanup."""
    script_path, actual_env = _prepare_execution(code, env)
    try:
        yield script_path, actual_env
    finally:
        Path(script_path).unlink(missing_ok=True)


@type_check
def run_python_code(
    code: str,
    env: dict[str, str] | None = None,
    config: RuntimeConfig | None = None,
) -> ExecutionResult:
    """Execute Python code in a subprocess."""
    if config is None:
        config = RuntimeConfig()

    logger.debug("runtime_execute", code_length=len(code), timeout=config.timeout_seconds)

    with _execution_context(code, env) as (script_path, actual_env):
        try:
            cmd = [config.python_executable, script_path]
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


@type_check
async def run_python_code_async(
    code: str,
    env: dict[str, str] | None = None,
    config: RuntimeConfig | None = None,
) -> ExecutionResult:
    """Execute Python code asynchronously in a subprocess."""
    if config is None:
        config = RuntimeConfig()

    logger.debug(
        "runtime_execute_async",
        code_length=len(code),
        timeout=config.timeout_seconds,
    )

    with _execution_context(code, env) as (script_path, actual_env):
        try:
            process = await asyncio.create_subprocess_exec(
                config.python_executable,
                script_path,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                env=actual_env,
                cwd=config.working_directory,
            )

            try:
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
                process.kill()
                await process.wait()
                logger.warning("runtime_execute_async_timeout", timeout=config.timeout_seconds)
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
