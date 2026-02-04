import logging
import subprocess
import sys
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# Add project root to path so we can import models if they are mounted
sys.path.append("/app")

try:
    from src.environment.models import (
        CommandRequest,
        CommandResponse,
        ScriptRequest,
        ToolRequest,
        ToolResponse,
    )
except ImportError:
    # Fallback if imports fail (e.g. during bootstrapping or different path structure)
    # We redefine simple models to ensure functionality
    from pydantic import Field

    class ToolRequest(BaseModel):
        tool_name: str
        arguments: dict[str, Any] = {}

    class ToolResponse(BaseModel):
        output: str
        error: str | None = None
        status: str = "success"

    class CommandRequest(BaseModel):
        command: str
        timeout: int = 30
        workdir: str | None = None

    class CommandResponse(BaseModel):
        stdout: str
        stderr: str
        return_code: int

    class ScriptRequest(BaseModel):
        script_name: str
        args: list[str] = []
        timeout: int = 30


app = FastAPI(title="Problemologist Container Agent")
logger = logging.getLogger("uvicorn.error")


@app.get("/health")
def health_check():
    return {"status": "ok"}


@app.post("/exec", response_model=ToolResponse)
def execute_tool(request: ToolRequest):
    """
    Placeholder for tool execution if we move tool logic inside container.
    """
    return ToolResponse(
        output=f"Tool {request.tool_name} not implemented in agent yet."
    )


@app.post("/run_command", response_model=CommandResponse)
def run_command(request: CommandRequest):
    workdir = Path(request.workdir) if request.workdir else Path("/workspace")
    try:
        workdir.mkdir(parents=True, exist_ok=True)

        logger.info(f"Running command: {request.command}")
        result = subprocess.run(
            request.command,
            shell=True,
            capture_output=True,
            text=True,
            cwd=workdir,
            timeout=request.timeout,
        )
        return CommandResponse(
            stdout=result.stdout,
            stderr=result.stderr,
            return_code=result.returncode,
        )
    except subprocess.TimeoutExpired:
        return CommandResponse(
            stdout="",
            stderr=f"Command timed out after {request.timeout}s",
            return_code=124,
        )
    except Exception as e:
        logger.error(f"Error running command: {e}")
        return CommandResponse(
            stdout="",
            stderr=str(e),
            return_code=1,
        )


@app.post("/run_script", response_model=CommandResponse)
def run_script(request: ScriptRequest):
    workdir = Path("/workspace")
    script_path = workdir / request.script_name

    if not script_path.exists():
        return CommandResponse(
            stdout="",
            stderr=f"Script {request.script_name} not found.",
            return_code=1,
        )

    cmd = [sys.executable, request.script_name, *request.args]

    try:
        logger.info(f"Running script: {cmd}")
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            cwd=workdir,
            timeout=request.timeout,
        )
        return CommandResponse(
            stdout=result.stdout,
            stderr=result.stderr,
            return_code=result.returncode,
        )
    except subprocess.TimeoutExpired:
        return CommandResponse(
            stdout="",
            stderr=f"Script timed out after {request.timeout}s",
            return_code=124,
        )
    except Exception as e:
        logger.error(f"Error running script: {e}")
        return CommandResponse(
            stdout="",
            stderr=str(e),
            return_code=1,
        )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
