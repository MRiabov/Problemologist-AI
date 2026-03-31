from typing import Any, Literal, TypeAlias

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    StrictBool,
    StrictInt,
    StrictStr,
    field_validator,
)

from shared.enums import AgentName, AssetType, EpisodeStatus, ResponseStatus
from shared.models.schemas import BenchmarkPartAttachmentPolicy, ElectronicsSection
from shared.models.simulation import (
    FluidMetricResult,
    MultiRunResult,
    SimulationFailure,
    StressSummary,
)
from shared.observability.schemas import BaseEvent
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.simulation.smoke_mode import ensure_smoke_test_mode_allowed
from shared.workers.workbench_models import ManufacturingMethod

ReviewerStage: TypeAlias = Literal[
    "benchmark_reviewer",
    "engineering_execution_reviewer",
    "electronics_reviewer",
]

LEGACY_REVIEWER_STAGE_ALIASES = {
    "engineer_execution_reviewer": "engineering_execution_reviewer",
}


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
    bypass_agent_permissions: bool = False


class FsFileEntry(BaseModel):
    """A single entry returned by the /fs/ls endpoint."""

    name: StrictStr
    path: StrictStr
    is_dir: bool
    size: int | None = None


class ReadFileRequest(BaseModel):
    """Request to read a file."""

    path: StrictStr = Field(..., min_length=1)
    bypass_agent_permissions: bool = False


class DeleteFileRequest(BaseModel):
    """Request to delete a file."""

    path: StrictStr = Field(..., min_length=1)
    bypass_agent_permissions: bool = False


class WriteFileRequest(BaseModel):
    """Request to write a file."""

    path: StrictStr = Field(..., min_length=1)
    content: StrictStr
    overwrite: bool = False  # was and should've been StrictBool?
    bypass_agent_permissions: bool = False


class EditOp(BaseModel):
    """A single edit operation (find and replace)."""

    old_string: StrictStr = Field(..., min_length=1)
    new_string: StrictStr


class EditFileRequest(BaseModel):
    """Request to edit a file with one or more operations."""

    path: StrictStr = Field(..., min_length=1)
    edits: list[EditOp] = Field(..., min_length=1)
    bypass_agent_permissions: bool = False


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
    bypass_agent_permissions: bool = False


class ExecuteRequest(BaseModel):
    """Request to execute a shell command in the session workspace."""

    code: StrictStr = Field(..., min_length=1)
    timeout: StrictInt = Field(default=30, ge=1, le=1000)
    episode_id: StrictStr | None = None


class ExecuteResponse(BaseModel):
    """Response from executing a shell command."""

    stdout: str
    stderr: str
    exit_code: int
    timed_out: bool = False
    events: list[BaseEvent] = Field(default_factory=list)


class ScriptExecutionRequest(BaseModel):
    """Internal request for shell-command execution with session context."""

    code: StrictStr
    session_id: StrictStr
    timeout: StrictInt = 30
    episode_id: StrictStr | None = None


class BenchmarkToolRequest(BaseModel):
    """Request to run a benchmark tool (simulate, validate, etc.)."""

    script_path: StrictStr = Field(
        default="script.py",
        description=(
            "Path to the authored script. The runtime accepts either "
            "build() or a module-level final assembly such as `result = ...`."
        ),
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
        default_factory=get_default_simulator_backend,
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
    reviewer_stage: ReviewerStage | None = Field(
        default=None,
        description="Reviewer stage when calling /benchmark/submit.",
    )
    benchmark_episode_id: StrictStr | None = None
    benchmark_worker_session_id: StrictStr | None = None
    benchmark_revision: StrictStr | None = None
    episode_id: StrictStr | None = None

    @field_validator("backend", mode="before")
    @classmethod
    def normalize_backend(cls, value: object) -> object:
        if isinstance(value, str):
            return value.upper()
        return value

    @field_validator("reviewer_stage", mode="before")
    @classmethod
    def normalize_reviewer_stage(cls, value: object) -> object:
        if isinstance(value, AgentName):
            value = value.value
        if isinstance(value, str):
            return LEGACY_REVIEWER_STAGE_ALIASES.get(value, value)
        return value


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
    """Request to run batched runtime-randomization verification."""

    jitter_range: tuple[float, float, float] = Field(
        default=(0.002, 0.002, 0.001),
        description="(x, y, z) position jitter range in meters.",
    )
    num_scenes: int = Field(
        default=5,
        ge=1,
        description="Number of parallel jittered scene instances in one backend run.",
    )
    duration: float = Field(default=10.0, ge=0.1, description="Duration per run.")
    seed: int = Field(default=42, description="Random seed.")


class ElectronicsValidationRequest(BaseModel):
    """Request to validate an electronic circuit."""

    section: ElectronicsSection


class SimulationArtifacts(BaseModel):
    """Structured artifacts from a simulation run."""

    render_paths: list[StrictStr] = Field(default_factory=list)
    render_blobs_base64: dict[StrictStr, StrictStr] = Field(default_factory=dict)
    mjcf_content: StrictStr | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    circuit_validation_result: dict[StrictStr, Any] | None = None
    scene_path: StrictStr | None = None
    failure: SimulationFailure | None = None
    verification_result: MultiRunResult | None = None
    total_cost: float | None = None
    total_weight_g: float | None = None
    validation_results_json: StrictStr | None = None
    simulation_result_json: StrictStr | None = None
    review_manifest_json: StrictStr | None = None
    review_manifests_json: dict[StrictStr, StrictStr] = Field(default_factory=dict)

    model_config = {"extra": "allow"}


class BenchmarkToolResponse(BaseModel):
    """Response from a benchmark tool."""

    success: bool
    message: str
    confidence: Literal["low", "medium", "high", "approximate"] = "high"
    artifacts: SimulationArtifacts | None = None
    events: list[BaseEvent] = Field(default_factory=list)


class ValidationResultRecord(BaseModel):
    """Persisted validation gate result for submission handoff checks."""

    success: bool
    message: str | None = None
    timestamp: float
    script_path: StrictStr | None = None
    script_sha256: StrictStr | None = None
    verification_result: MultiRunResult | None = None

    model_config = {"extra": "forbid"}


class ReviewManifest(BaseModel):
    """Persisted handoff manifest used to gate reviewer entry."""

    status: Literal["ready_for_review"]
    reviewer_stage: ReviewerStage
    timestamp: str | None = None
    session_id: StrictStr
    revision: StrictStr | None = None
    episode_id: StrictStr | None = None
    worker_session_id: StrictStr | None = None
    benchmark_episode_id: StrictStr | None = None
    benchmark_worker_session_id: StrictStr | None = None
    benchmark_revision: StrictStr | None = None
    solution_revision: StrictStr | None = None
    environment_version: StrictStr | None = None
    preview_evidence_paths: list[StrictStr] = Field(default_factory=list)
    script_path: StrictStr
    script_sha256: StrictStr
    validation_success: bool
    validation_timestamp: float
    simulation_success: bool
    simulation_summary: StrictStr
    simulation_timestamp: float
    goal_reached: bool
    renders: list[StrictStr] = Field(default_factory=list)
    benchmark_attachment_policy_summary: list["BenchmarkAttachmentPolicySummary"] = (
        Field(default_factory=list)
    )
    mjcf_path: StrictStr | None = None
    cad_path: StrictStr | None = None
    objectives_path: StrictStr | None = None
    assembly_definition_path: StrictStr | None = None

    model_config = ConfigDict(extra="forbid")


class BenchmarkAttachmentPolicySummary(BaseModel):
    """Structured summary of a benchmark-owned fixture attachment policy."""

    part_id: StrictStr
    label: StrictStr
    allows_engineer_interaction: StrictBool
    attachment_policy: BenchmarkPartAttachmentPolicy | None = None


class PlanReviewManifest(BaseModel):
    """Planner handoff manifest used to gate engineering plan reviewer entry."""

    status: Literal["ready_for_review"]
    reviewer_stage: Literal[
        AgentName.BENCHMARK_PLAN_REVIEWER,
        AgentName.ENGINEER_PLAN_REVIEWER,
    ]
    session_id: StrictStr
    planner_node_type: AgentName
    episode_id: StrictStr | None = None
    worker_session_id: StrictStr | None = None
    benchmark_revision: StrictStr | None = None
    environment_version: StrictStr | None = None
    artifact_hashes: dict[StrictStr, StrictStr]

    model_config = ConfigDict(extra="forbid")


class COTSReproducibilityManifest(BaseModel):
    """System-owned catalog provenance snapshot for the current workspace session."""

    catalog_version: StrictStr | None = None
    bd_warehouse_commit: StrictStr | None = None
    catalog_snapshot_id: StrictStr | None = None
    generated_at: StrictStr | None = None


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


WorkerLightRpcAction: TypeAlias = Literal[
    "fs_ls",
    "fs_exists",
    "fs_read",
    "fs_read_blob",
    "fs_write",
    "fs_edit",
    "fs_upload_file",
    "fs_delete",
    "fs_grep",
    "fs_bundle",
    "git_init",
    "git_commit",
    "git_status",
    "git_resolve",
    "git_merge_abort",
    "git_merge_complete",
    "runtime_execute",
    "topology_inspect",
]


class WorkerLightRpcRequest(BaseModel):
    """Typed request envelope for the worker-light websocket RPC transport."""

    request_id: StrictStr
    action: WorkerLightRpcAction
    payload: dict[StrictStr, Any] = Field(default_factory=dict)

    model_config = ConfigDict(extra="forbid")


class WorkerLightRpcError(BaseModel):
    """Structured error payload returned by worker-light websocket RPC."""

    message: StrictStr
    status_code: StrictInt | None = None
    error_type: StrictStr | None = None

    model_config = ConfigDict(extra="forbid")


class WorkerLightRpcResponse(BaseModel):
    """Typed response envelope for the worker-light websocket RPC transport."""

    request_id: StrictStr
    ok: StrictBool
    result: Any | None = None
    error: WorkerLightRpcError | None = None

    model_config = ConfigDict(extra="forbid")


class PreviewDesignRequest(BaseModel):
    """Request to preview a CAD design from specific angles."""

    script_path: StrictStr = Field(
        default="script.py",
        description=(
            "Path to the authored script. The runtime accepts either "
            "build() or a module-level final assembly such as `result = ...`."
        ),
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

    @field_validator("smoke_test_mode", mode="after")
    @classmethod
    def validate_smoke_test_mode(cls, value: bool | None) -> bool | None:
        return ensure_smoke_test_mode_allowed(value)


class PreviewDesignResponse(BaseModel):
    """Response from preview design endpoint."""

    success: StrictBool
    message: StrictStr
    image_path: StrictStr | None = None
    image_bytes_base64: StrictStr | None = Field(
        default=None,
        description=(
            "Base64-encoded preview image payload used internally by worker "
            "delegation layers."
        ),
    )
    events: list[BaseEvent] = Field(default_factory=list)


class HeavySimulationParams(BaseModel):
    """Parameters for worker_run_simulation activity."""

    bundle_base64: StrictStr
    script_path: str
    backend: SimulatorBackendType
    smoke_test_mode: bool | None = None
    session_id: str


class HeavyValidationParams(BaseModel):
    """Parameters for worker_validate_design activity."""

    bundle_base64: StrictStr
    script_path: str
    session_id: str
    smoke_test_mode: bool | None = None


class HeavyVerifyParams(VerificationRequest):
    """Parameters for worker_verify_design activity."""

    session_id: str


class HeavyPreviewParams(BaseModel):
    """Parameters for worker_preview_design activity."""

    bundle_base64: StrictStr
    script_path: str
    pitch: float = -45.0
    yaw: float = 45.0


class SimulationVideoRequest(BaseModel):
    """Request to encode simulation frames into a video artifact."""

    bundle_base64: StrictStr
    frame_paths: list[StrictStr] = Field(default_factory=list)
    output_name: StrictStr = "simulation.mp4"
    fps: StrictInt = Field(default=30, ge=1, le=240)
    session_id: StrictStr | None = None


class HeavySubmitParams(BaseModel):
    """Parameters for worker_submit_for_review activity."""

    bundle_base64: StrictStr
    script_path: str
    reviewer_stage: ReviewerStage
    session_id: str
    episode_id: str | None = None
    benchmark_episode_id: str | None = None
    benchmark_worker_session_id: str | None = None
    benchmark_revision: str | None = None


class HeavyValidationResponse(BaseModel):
    """Response from worker_validate_design activity."""

    success: bool
    message: str | None = None


class HeavyPreviewResponse(BaseModel):
    """Response from worker_preview_design activity."""

    success: bool
    image_bytes: bytes | None = None
    image_path: str | None = None
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


class RenderSiblingPaths(BaseModel):
    """Sibling render artifact paths for a single camera/view group."""

    rgb: StrictStr | None = None
    depth: StrictStr | None = None
    segmentation: StrictStr | None = None

    model_config = ConfigDict(extra="forbid")


class SegmentationLegendEntry(BaseModel):
    """One segmentation color entry exposed to media inspection."""

    instance_id: StrictStr
    instance_name: StrictStr
    semantic_label: StrictStr
    object_type: StrictStr
    object_id: StrictInt
    body_name: StrictStr | None = None
    geom_name: StrictStr | None = None
    color_rgb: tuple[int, int, int]
    color_hex: StrictStr


class RenderArtifactMetadata(BaseModel):
    """Structured metadata persisted for one render artifact."""

    modality: Literal["rgb", "depth", "segmentation", "unknown"] = "unknown"
    group_key: StrictStr | None = None
    siblings: RenderSiblingPaths = Field(default_factory=RenderSiblingPaths)
    depth_min_m: float | None = None
    depth_max_m: float | None = None
    depth_interpretation: StrictStr | None = None
    segmentation_legend: list[SegmentationLegendEntry] = Field(default_factory=list)

    model_config = ConfigDict(extra="forbid")


class RenderManifest(BaseModel):
    """Manifest persisted alongside rendered artifacts under renders/."""

    version: StrictStr = "1.0"
    episode_id: StrictStr | None = None
    worker_session_id: StrictStr | None = None
    revision: StrictStr | None = None
    environment_version: StrictStr | None = None
    preview_evidence_paths: list[StrictStr] = Field(default_factory=list)
    artifacts: dict[StrictStr, RenderArtifactMetadata] = Field(default_factory=dict)

    model_config = ConfigDict(extra="forbid")


class MediaInspectionResult(BaseModel):
    """Structured result for agent-facing visual media inspection."""

    path: StrictStr
    mime_type: StrictStr
    media_kind: Literal["image", "video", "video_frames", "unsupported"]
    attached_to_model: bool = False
    attached_media_count: StrictInt = Field(default=0, ge=0)
    size_bytes: StrictInt = Field(ge=0)
    note: StrictStr
    render_metadata: RenderArtifactMetadata | None = None
    data_url: StrictStr | None = Field(default=None, exclude=True, repr=False)
    data_urls: list[StrictStr] = Field(default_factory=list, exclude=True, repr=False)


class PlanRefusal(BaseModel):
    """Information about a refused plan."""

    status: str = "plan_refused"
    reason: str
    timestamp: str | None = None
    session_id: str = "default"


ReviewManifest.model_rebuild()
