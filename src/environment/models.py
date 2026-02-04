from enum import Enum
from typing import Any

from pydantic import BaseModel, Field


class ToolName(str, Enum):
    write = "write_file"
    edit = "edit_file"
    preview_design = "preview_design"
    verify_solution = "verify_solution"
    search_docs = "search_docs"
    search_parts = "search_parts"
    preview_part = "preview_part"
    analyze_design = "analyze_design"
    run_command = "run_command"
    lint_script = "lint_script"
    start_session = "start_session"
    stop_session = "stop_session"
    # Fallback/System tools
    view_file = "view_file"


class ToolRequest(BaseModel):
    """Request payload for executing a tool."""

    tool_name: ToolName = Field(..., description="Name of the tool to execute")
    arguments: dict[str, Any] = Field(
        default_factory=dict, description="Arguments for the tool"
    )


class ToolResponse(BaseModel):
    """Response payload from a tool execution."""

    output: str = Field(..., description="String output from the tool")
    error: str | None = Field(None, description="Error message if execution failed")
    status: str = Field("success", description="Status of the execution: success/error")


class CommandRequest(BaseModel):
    """Request payload for executing a shell command."""

    command: str = Field(..., description="Shell command to execute")
    timeout: int = Field(30, description="Timeout in seconds")
    workdir: str | None = Field(None, description="Working directory")


class CommandResponse(BaseModel):
    """Response payload from a command execution."""

    stdout: str = Field("", description="Standard output")
    stderr: str = Field("", description="Standard error")
    return_code: int = Field(..., description="Exit code")


class ScriptRequest(BaseModel):
    """Request payload for executing a python script."""

    script_name: str = Field(..., description="Name of the script in the workspace")
    args: list[str] = Field(default_factory=list, description="Command line arguments")
    timeout: int = Field(30, description="Timeout in seconds")


class JobStatus(str, Enum):
    queued = "queued"
    running = "running"
    completed = "completed"
    failed = "failed"
    cancelled = "cancelled"


class JobRequest(BaseModel):
    """Unified request for async jobs."""

    type: str = Field(..., description="Type of job: 'command' or 'script'")
    command: str | None = Field(None, description="Command to run (if type=command)")
    script_name: str | None = Field(None, description="Script to run (if type=script)")
    args: list[str] = Field(default_factory=list, description="Arguments for script")
    workdir: str | None = Field(None, description="Working directory")
    timeout: int = Field(30, description="Timeout in seconds")


class JobResponse(BaseModel):
    """Response for job status polling."""

    job_id: str = Field(..., description="Unique job identifier")
    status: JobStatus = Field(..., description="Current status")
    stdout: str = Field("", description="Standard output so far")
    stderr: str = Field("", description="Standard error so far")
    return_code: int | None = Field(None, description="Exit code if completed")
    error: str | None = Field(None, description="Error message if failed")
