from typing import Any, Literal

from pydantic import BaseModel, Field, StrictBool, StrictInt, StrictStr

from shared.enums import ResponseStatus
from shared.simulation.schemas import SimulatorBackendType
from worker.workbenches.models import ManufacturingMethod


class ReadFileResponse(BaseModel):
    """Response from reading a file."""

    content: StrictStr


class ExistsResponse(BaseModel):
    """Response for file existence check."""

    exists: StrictBool


class StatusResponse(BaseModel):
    """Generic status response."""

    status: ResponseStatus


class ListFilesRequest(BaseModel):
    """Request to list files in a directory."""

    path: StrictStr = Field(default="/")


class ReadFileRequest(BaseModel):
    """Request to read a file."""

    path: StrictStr = Field(..., min_length=1)


class DeleteFileRequest(BaseModel):
    """Request to delete a file."""

    path: StrictStr = Field(..., min_length=1)


class WriteFileRequest(BaseModel):
    """Request to write a file."""

    path: StrictStr = Field(..., min_length=1)
    content: StrictStr
    overwrite: bool = False  # was and should've been StrictBool?


class EditOp(BaseModel):
    """A single edit operation (find and replace)."""

    old_string: StrictStr = Field(..., min_length=1)
    new_string: StrictStr


class EditFileRequest(BaseModel):
    """Request to edit a file with one or more operations."""

    path: StrictStr = Field(..., min_length=1)
    edits: list[EditOp] = Field(..., min_length=1)


class GrepMatch(BaseModel):
    """A single match from a grep operation."""

    path: StrictStr
    line: StrictInt
    text: StrictStr


class GrepRequest(BaseModel):
    """Request to search for a pattern in files."""

    pattern: StrictStr = Field(..., min_length=1)
    path: StrictStr | None = None
    glob: StrictStr | None = None


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
    events: list[dict[str, Any]] = Field(default_factory=list)


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
    backend: SimulatorBackendType = Field(
        default=SimulatorBackendType.GENESIS,
        description="Physics backend to use.",
    )
    smoke_test_mode: bool = Field(
        default=False,
        description="If true: cap particles to 5000, label results as approximate.",
    )
    particle_budget: int | None = Field(
        default=None,
        description="Optional particle budget override.",
    )


class AnalyzeRequest(BenchmarkToolRequest):
    """Request to run manufacturing analysis."""

    method: ManufacturingMethod = Field(
        ...,
        description="Manufacturing method to analyze (cnc, 3dp, im).",
    )
    quantity: StrictInt = Field(
        default=1,
        ge=1,
        description="Number of units to manufacture for pricing.",
    )


class BenchmarkToolResponse(BaseModel):
    """Response from a benchmark tool."""

    success: StrictBool
    message: StrictStr
    confidence: StrictStr = "high"
    artifacts: dict[StrictStr, Any] | None = None
    events: list[dict[str, Any]] = Field(default_factory=list)


class GitCommitRequest(BaseModel):
    """Request to commit changes in the session workspace."""

    message: StrictStr = Field(..., min_length=1)


class GitCommitResponse(BaseModel):
    """Response from a git commit operation."""

    success: StrictBool
    commit_hash: StrictStr | None = None
    message: StrictStr


class GitStatusResponse(BaseModel):
    """Response from git status."""

    branch: StrictStr
    is_dirty: StrictBool
    is_merging: StrictBool
    conflicts: list[StrictStr] = Field(default_factory=list)
    error: StrictStr | None = None


class GitResolveRequest(BaseModel):
    """Request to resolve a merge conflict."""

    file_path: StrictStr = Field(..., min_length=1)
    strategy: Literal["ours", "theirs"]


class GitMergeRequest(BaseModel):
    """Request to complete a merge."""

    message: StrictStr | None = None


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
    events: list[dict[str, Any]] = Field(default_factory=list)


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


class InspectTopologyRequest(BaseModel):
    """Request to inspect topological features."""

    target_id: StrictStr = Field(..., description="Target ID (e.g. face_0, part_1).")
    script_path: StrictStr = Field(
        default="script.py", description="Path to the build script."
    )


class InspectTopologyResponse(BaseModel):
    """Response from topology inspection."""

    success: StrictBool
    properties: dict[StrictStr, Any] | None = None
    message: StrictStr | None = None
