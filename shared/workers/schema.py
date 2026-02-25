from typing import Any, Literal

from pydantic import BaseModel, Field, StrictBool, StrictInt, StrictStr

from shared.enums import AssetType, EpisodeStatus, ResponseStatus
from shared.models.schemas import ElectronicsSection
from shared.models.simulation import (
    FluidMetricResult,
    MultiRunResult,
    SimulationFailure,
    StressSummary,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.workbench_models import ManufacturingMethod


class ReadFileResponse(BaseModel):
    """Response from reading a file."""

    content: StrictStr


class ExistsResponse(BaseModel):
    """Response for file existence check."""

    exists: StrictBool


class StatusResponse(BaseModel):
    """Generic status response."""

    status: ResponseStatus


class HealthResponse(BaseModel):
    """Health payload returned by controller/worker `/health` endpoints."""

    status: ResponseStatus

    @field_validator("status", mode="before")
    @classmethod
    def normalize_status(cls, value: object) -> object:
        if isinstance(value, str):
            return value.upper()
        return value


class FailureSimulationConfig(BaseModel):
    """Configuration for simulating failures in activities."""

    mjcf_compilation: bool = False
    run_simulation: bool = False
    render_video: bool = False
    s3_upload: bool = False


class SimulationWorkflowParams(BaseModel):
    """Parameters for the simulation workflow."""

    compound_json: str
    episode_id: str
    simulate_failures: FailureSimulationConfig = Field(
        default_factory=FailureSimulationConfig
    )


class UpdateTraceParams(BaseModel):
    """Parameters for update_trace_activity."""

    episode_id: str
    s3_path: str
    asset_type: AssetType = AssetType.VIDEO
    status: EpisodeStatus = EpisodeStatus.COMPLETED
    error: str | None = None


class ListFilesRequest(BaseModel):
    """Request to list files in a directory."""

    path: StrictStr = Field(default="/")


class FsFileEntry(BaseModel):
    """A single entry returned by the /fs/ls endpoint."""

    name: StrictStr
    path: StrictStr
    is_dir: bool
    size: int | None = None


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

    stdout: str
    stderr: str
    exit_code: int
    timed_out: bool = False
    events: list[dict[str, Any]] = Field(default_factory=list)
    # fixme: don't events have a model?


class ScriptExecutionRequest(BaseModel):
    """Internal request for script execution with session context."""

    code: StrictStr
    session_id: StrictStr
    timeout: StrictInt = 30


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
    bundle_base64: str | None = Field(
        default=None,
        description="Gzipped tarball of the session workspace (base64 encoded).",
    )
    backend: SimulatorBackendType = Field(
        default=SimulatorBackendType.GENESIS,
        description="Physics backend to use.",
    )
    smoke_test_mode: bool | None = Field(
        default=None,
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


class VerificationRequest(BenchmarkToolRequest):
    """Request to run multi-seed verification with jitter."""

    jitter_range: tuple[float, float, float] = Field(
        default=(0.002, 0.002, 0.001),
        description="(x, y, z) position jitter range in meters.",
    )
    num_runs: int = Field(default=5, ge=1, description="Number of runs.")
    duration: float = Field(default=10.0, ge=0.1, description="Duration per run.")
    seed: int = Field(default=42, description="Random seed.")


class ElectronicsValidationRequest(BaseModel):
    """Request to validate an electronic circuit."""

    section: "ElectronicsSection"


class SimulationArtifacts(BaseModel):
    """Structured artifacts from a simulation run."""

    render_paths: list[StrictStr] = Field(default_factory=list)
    mjcf_content: StrictStr | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    circuit_validation_result: dict[StrictStr, Any] | None = None
    scene_path: StrictStr | None = None
    failure: SimulationFailure | None = None
    verification_result: MultiRunResult | None = None

    model_config = {"extra": "allow"}


class BenchmarkToolResponse(BaseModel):
    """Response from a benchmark tool."""

    success: bool
    message: str
    confidence: str = "high"
    artifacts: SimulationArtifacts | dict[str, Any] | None = None
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
    script_content: StrictStr | None = Field(
        default=None,
        description="Direct content of the script.",
    )
    bundle_base64: str | None = Field(
        default=None,
        description="Gzipped tarball of the session workspace (base64 encoded).",
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
    smoke_test_mode: bool | None = Field(
        default=None,
        description="If true: cap particles to 5000, label results as approximate.",
    )


class PreviewDesignResponse(BaseModel):
    """Response from preview design endpoint."""

    success: StrictBool
    message: StrictStr
    image_path: StrictStr | None = None
    events: list[dict[str, Any]] = Field(default_factory=list)


class HeavySimulationParams(BaseModel):
    """Parameters for worker_run_simulation activity."""

    bundle_bytes: bytes
    script_path: str
    backend: SimulatorBackendType
    smoke_test_mode: bool | None = None
    session_id: str


class HeavyValidationParams(BaseModel):
    """Parameters for worker_validate_design activity."""

    bundle_bytes: bytes
    script_path: str
    session_id: str
    smoke_test_mode: bool | None = None


class HeavyPreviewParams(BaseModel):
    """Parameters for worker_preview_design activity."""

    bundle_bytes: bytes
    script_path: str
    pitch: float = -45.0
    yaw: float = 45.0


class HeavyValidationResponse(BaseModel):
    """Response from worker_validate_design activity."""

    success: bool
    message: str | None = None


class HeavyPreviewResponse(BaseModel):
    """Response from worker_preview_design activity."""

    success: bool
    image_bytes: bytes | None = None
    filename: str | None = None


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
class PlanRefusal(BaseModel):
    """Information about a refused plan."""

    status: str = "plan_refused"
    reason: str
    timestamp: str | None = None
    session_id: str = "default"
