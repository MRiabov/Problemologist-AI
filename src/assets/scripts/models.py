from enum import StrEnum

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


class JobRequest(BaseModel):
    """
    Unified request for async jobs.
    """

    type: JobType = Field(..., description="Type of job: 'command' or 'script'")
    command: str | None = Field(None, description="Command to run (if type=command)")
    script_name: str | None = Field(None, description="Script to run (if type=script)")
    args: list[str] = Field(default_factory=list, description="Arguments for script")
    workdir: str | None = Field(None, description="Working directory")
    timeout: int = Field(30, strict=True, description="Timeout in seconds")


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


