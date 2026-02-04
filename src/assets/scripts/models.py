from enum import StrEnum
from typing import Any
from pydantic import BaseModel, Field


class JobType(StrEnum):
    COMMAND = "command"
    SCRIPT = "script"


class JobStatus(StrEnum):
    QUEUED = "queued"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class ToolName(StrEnum):
    WRITE_FILE = "write_file"
    EDIT_FILE = "edit_file"
    PREVIEW_DESIGN = "preview_design"
    VERIFY_SOLUTION = "verify_solution"
    SEARCH_DOCS = "search_docs"
    SEARCH_PARTS = "search_parts"
    PREVIEW_PART = "preview_part"
    ANALYZE_DESIGN = "analyze_design"
    RUN_COMMAND = "run_command"
    LINT_SCRIPT = "lint_script"
    START_SESSION = "start_session"
    STOP_SESSION = "stop_session"
    VIEW_FILE = "view_file"


class JobRequest(BaseModel):
    """
    Unified request for async jobs.
    """

    type: JobType = Field(..., description="Type of job: 'command' or 'script'")
    command: str | None = Field(None, description="Command to run (if type=command)")
    script_name: str | None = Field(None, description="Script to run (if type=script)")
    args: list[str] = Field(default_factory=list, description="Arguments for script")
    workdir: str | None = Field(None, description="Working directory")
    timeout: int = Field(30, description="Timeout in seconds")


class JobResponse(BaseModel):
    """
    Response for job status polling.
    """

    job_id: str = Field(..., description="Unique job identifier")
    status: JobStatus = Field(..., description="Current status")
    stdout: str = Field("", description="Standard output so far")
    stderr: str = Field("", description="Standard error so far")
    return_code: int | None = Field(None, description="Exit code if completed")
    error: str | None = Field(None, description="Error message if failed")


class ToolRequest(BaseModel):
    """
    Request payload for executing a tool.
    """

    tool_name: ToolName = Field(..., description="Name of the tool to execute")
    arguments: dict[str, Any] = Field(
        default_factory=dict, description="Arguments for the tool"
    )


class ToolExecutionStatus(StrEnum):
    SUCCESS = "success"
    ERROR = "error"


class ToolResponse(BaseModel):
    """
    Response payload from a tool execution.
    """

    output: str = Field(..., description="String output from the tool")
    error: str | None = Field(None, description="Error message if execution failed")
    status: ToolExecutionStatus = Field(
        ToolExecutionStatus.SUCCESS,
        description="Status of the execution: success/error",
    )
