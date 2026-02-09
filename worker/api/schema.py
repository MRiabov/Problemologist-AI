from typing import Any

from pydantic import BaseModel, Field, StrictBool, StrictInt, StrictStr

from shared.enums import ResponseStatus


class ReadFileResponse(BaseModel):
    """Response from reading a file."""

    content: StrictStr


class StatusResponse(BaseModel):
    """Generic status response."""

    status: ResponseStatus


class ListFilesRequest(BaseModel):
    """Request to list files in a directory."""

    path: StrictStr = Field(default="/", min_length=1)


class ReadFileRequest(BaseModel):
    """Request to read a file."""

    path: StrictStr = Field(..., min_length=1)


class WriteFileRequest(BaseModel):
    """Request to write a file."""

    path: StrictStr = Field(..., min_length=1)
    content: StrictStr


class EditOp(BaseModel):
    """A single edit operation (find and replace)."""

    old_string: StrictStr = Field(..., min_length=1)
    new_string: StrictStr


class EditFileRequest(BaseModel):
    """Request to edit a file with one or more operations."""

    path: StrictStr = Field(..., min_length=1)
    edits: list[EditOp] = Field(..., min_length=1)


class ExecuteRequest(BaseModel):
    """Request to execute Python code."""

    code: StrictStr = Field(..., min_length=1)
    timeout: StrictInt = Field(default=30, ge=1, le=300)


class ExecuteResponse(BaseModel):
    """Response from executing Python code."""

    stdout: StrictStr
    stderr: StrictStr
    exit_code: StrictInt
    timed_out: StrictBool = False


class BenchmarkToolRequest(BaseModel):
    """Request to run a benchmark tool (simulate, validate, etc.)."""

    script_path: StrictStr = Field(
        default="script.py",
        description="Path to the script containing the build() function.",
    )
    script_content: StrictStr | None = Field(
        default=None,
        description="Direct content of the script.",
    )


class BenchmarkToolResponse(BaseModel):
    """Response from a benchmark tool."""

    success: StrictBool
    message: StrictStr
    artifacts: dict[StrictStr, Any] | None = None


class GitCommitRequest(BaseModel):
    """Request to commit changes in the session workspace."""

    message: StrictStr = Field(..., min_length=1)


class GitCommitResponse(BaseModel):
    """Response from a git commit operation."""

    success: StrictBool
    commit_hash: StrictStr | None = None
    message: StrictStr


class PreviewDesignRequest(BaseModel):
    """Request to preview a CAD design from specific angles."""

    script_path: StrictStr = Field(
        default="script.py",
        description="Path to the script containing the build() function.",
    )
    pitch: float = Field(
        default=-45.0,
        ge=-90.0,
        le=90.0,
        description="Camera elevation angle in degrees (negative = looking down).",
    )
    yaw: float = Field(
        default=45.0,
        ge=0.0,
        lt=360.0,
        description="Camera azimuth angle in degrees (clockwise from front).",
    )


class PreviewDesignResponse(BaseModel):
    """Response from preview design endpoint."""

    success: StrictBool
    message: StrictStr
    image_path: StrictStr | None = None


class LintRequest(BaseModel):
    """Request to lint Python code."""

    path: StrictStr | None = Field(
        default=None,
        description="Path to file to lint (if in filesystem).",
    )
    content: StrictStr | None = Field(
        default=None,
        description="Direct content to lint.",
    )


class LintResponse(BaseModel):
    """Response from linting Python code."""

    success: StrictBool
    errors: list[dict[StrictStr, Any]] = Field(default_factory=list)
    warnings: list[dict[StrictStr, Any]] = Field(default_factory=list)


class GrepRequest(BaseModel):
    """Request to search for a pattern in files."""

    pattern: StrictStr = Field(..., min_length=1)
    path: StrictStr | None = Field(default=None, description="Path to search in (file or directory).")
    glob: StrictStr | None = Field(default=None, description="Glob pattern to filter files.")


class GrepMatchModel(BaseModel):
    """A single match from a grep operation."""

    path: StrictStr
    line: StrictInt
    text: StrictStr


class GrepResponse(BaseModel):
    """Response from a grep operation."""

    matches: list[GrepMatchModel] = Field(default_factory=list)
    error: StrictStr | None = None


class GlobRequest(BaseModel):
    """Request to find files matching a pattern."""

    pattern: StrictStr = Field(..., min_length=1)
    path: StrictStr = Field(default="/", description="Base path to search from.")
