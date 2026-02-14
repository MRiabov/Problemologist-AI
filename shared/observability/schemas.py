from datetime import UTC, datetime
from enum import StrEnum
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class ObservabilityEventType(StrEnum):
    # 1. Component usage
    COMPONENT_USAGE = "component_usage"
    # 2. Tool invocation
    TOOL_INVOCATION = "tool_invocation"
    # 3. Manufacturability and price check (engineer)
    MANUFACTURABILITY_CHECK = "manufacturability_check"
    # 4. Scene valiation (Benchmark CAD engineer)
    SCENE_VALIDATION = "scene_validation"
    # 5. Render request (engineer)
    RENDER_REQUEST_ENGINEER = "render_request_engineer"
    # 6. Render request (benchmark)
    RENDER_REQUEST_BENCHMARK = "render_request_benchmark"
    # 7. Simulation request (engineer)
    SIMULATION_REQUEST = "simulation_request"
    # 8. Simulation result (engineer)
    SIMULATION_RESULT = "simulation_result"
    # 9. COTS search (engineer/planner?)
    COTS_SEARCH = "cots_search"
    # 10. Plan submission (benchmark)
    PLAN_SUBMISSION_BENCHMARK = "plan_submission_benchmark"
    # 11. Plan submission (Engineer)
    PLAN_SUBMISSION_ENGINEER = "plan_submission_engineer"
    # 12. Price/weight failure escalation request (CAD engineer)
    ESCALATION_REQUEST = "escalation_request"
    # 13. Price/weight failure escalation decision (reviewer)
    ESCALATION_DECISION = "escalation_decision"
    # 14. Lint failure - code
    LINT_FAILURE_CODE = "lint_failure_code"
    # 15. Lint failure - Markdown/YAML
    LINT_FAILURE_DOCS = "lint_failure_docs"
    # 16. Logic/constraint failure - YAML
    LOGIC_FAILURE = "logic_failure"
    # 17. Skill edit (skill editing agent)
    SKILL_EDIT = "skill_edit"
    # 18. Skill read (used as context)
    SKILL_READ = "skill_read"
    # 19. Tool-specific events for easier navigation/aggregation
    TOOL_LS_FILES = "ls_files_tool"
    TOOL_GREP = "grep_tool"
    TOOL_READ_FILE = "read_file_tool"
    TOOL_WRITE_FILE = "write_file_tool"
    TOOL_EDIT_FILE = "edit_files_tool"
    TOOL_RUN_COMMAND = "run_command_tool"
    TOOL_GIT_INIT = "git_init_tool"
    TOOL_GIT_COMMIT = "git_commit_tool"

    # 20. Simulation instability
    SIMULATION_INSTABILITY = "simulation_instability"
    # 21. Submission validation
    SUBMISSION_VALIDATION = "submission_validation"
    # 22. Cost/weight delta heuristic
    COST_WEIGHT_DELTA = "cost_weight_delta"
    # 23. Library usage
    LIBRARY_USAGE = "library_usage"
    # 24. Review decision (full details)
    REVIEW_DECISION = "review_decision"

    # 25. WP3 Electronics events
    CIRCUIT_VALIDATION = "circuit_validation"
    WIRE_ROUTING = "wire_routing"
    POWER_BUDGET_CHECK = "power_budget_check"
    ELECTRICAL_FAILURE = "electrical_failure"
    ELEC_AGENT_HANDOVER = "elec_agent_handover"
    CIRCUIT_SIMULATION = "circuit_simulation"

    # 26. WP2 Fluids & Deformable Materials events
    SIMULATION_BACKEND_SELECTED = "simulation_backend_selected"
    PART_BREAKAGE = "part_breakage"
    FLUID_CONTAINMENT_CHECK = "fluid_containment_check"
    FLOW_RATE_CHECK = "flow_rate_check"
    STRESS_SUMMARY = "stress_summary"
    MESHING_FAILURE = "meshing_failure"
    PHYSICS_INSTABILITY = "physics_instability"
    GPU_OOM_RETRY = "gpu_oom_retry"


class SimulationFailureReason(StrEnum):
    TIMEOUT = "timeout"
    OUT_OF_BOUNDS = "out_of_bounds"
    FORBID_ZONE_HIT = "forbid_zone_hit"
    PART_BREAKAGE = "part_breakage"
    STABILITY_ISSUE = "stability_issue"
    NONE = "none"


class BaseEvent(BaseModel):
    """Base class for all observability events."""

    model_config = ConfigDict(use_enum_values=True)

    event_type: ObservabilityEventType
    timestamp: datetime = Field(default_factory=lambda: datetime.now(UTC))
    agent_id: str | None = None


class ComponentUsageEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.COMPONENT_USAGE
    category: str
    part_number: str
    label: str
    price: float
    weight_g: float


class ToolInvocationEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_INVOCATION
    tool_name: str
    arguments: dict[str, Any]


class ManufacturabilityCheckEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.MANUFACTURABILITY_CHECK
    part_id: str
    method: str  # e.g., "cnc", "injection_molding"
    result: bool  # pass/fail
    price: float | None = None
    weight_g: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneValidationEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SCENE_VALIDATION
    result: bool
    errors: list[str] = Field(default_factory=list)


class RenderRequestEngineerEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.RENDER_REQUEST_ENGINEER
    num_views: int = 24


class RenderRequestBenchmarkEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.RENDER_REQUEST_BENCHMARK
    num_views: int = 24


class SimulationRequestEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SIMULATION_REQUEST
    script_path: str


class SimulationResultEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SIMULATION_RESULT
    success: bool
    failure_reason: SimulationFailureReason = SimulationFailureReason.NONE
    time_elapsed_s: float
    compute_time_ms: float
    metadata: dict[str, Any] = Field(default_factory=dict)


class COTSSearchEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.COTS_SEARCH
    query: str
    results_count: int


class PlanSubmissionBenchmarkEvent(BaseEvent):
    event_type: ObservabilityEventType = (
        ObservabilityEventType.PLAN_SUBMISSION_BENCHMARK
    )
    plan_path: str


class PlanSubmissionEngineerEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.PLAN_SUBMISSION_ENGINEER
    plan_path: str


class EscalationRequestEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.ESCALATION_REQUEST
    reason: str
    current_price: float | None = None
    current_weight: float | None = None


class EscalationDecisionEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.ESCALATION_DECISION
    decision: str  # "approved", "rejected"
    comments: list[str] = Field(default_factory=list)


class LintFailureCodeEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.LINT_FAILURE_CODE
    file_path: str
    errors: list[str]


class LintFailureDocsEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.LINT_FAILURE_DOCS
    file_path: str
    errors: list[str]


class LogicFailureEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.LOGIC_FAILURE
    file_path: str
    constraint_name: str
    error_message: str


class SkillEditEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SKILL_EDIT
    skill_name: str
    action: str  # "create", "update", "delete"
    lines_changed: int


class SkillReadEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SKILL_READ
    skill_path: str
    skill_name: str


class LsFilesToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_LS_FILES
    path: str


class GrepToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_GREP
    pattern: str
    path: str | None = None
    glob: str | None = None


class ReadFileToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_READ_FILE
    path: str


class WriteFileToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_WRITE_FILE
    path: str
    content_snippet: str | None = None  # First 100 chars or so


class EditFileToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_EDIT_FILE
    path: str
    num_edits: int


class RunCommandToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_RUN_COMMAND
    command: str


class GitInitToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_GIT_INIT


class GitCommitToolEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.TOOL_GIT_COMMIT
    message: str


class SimulationInstabilityEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SIMULATION_INSTABILITY
    instability_type: str  # "nan", "penetration", "joint_violation"
    part_ids: list[str] = Field(default_factory=list)
    value: float | None = None
    message: str | None = None


class SubmissionValidationEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.SUBMISSION_VALIDATION
    artifacts_present: list[str]
    verification_passed: bool
    reasoning_trace_quality: float = Field(..., ge=0.0, le=1.0)
    errors: list[str] = Field(default_factory=list)


class CostWeightDeltaEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.COST_WEIGHT_DELTA
    best_simulated_cost: float
    best_simulated_weight_g: float
    final_cost: float
    final_weight_g: float
    is_worse: bool


class LibraryUsageEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.LIBRARY_USAGE
    module_name: str
    usage_type: str  # "new", "reused"
    path: str


class ReviewDecisionEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.REVIEW_DECISION
    decision: str  # "approve", "reject_plan", "reject_code"
    reason: str
    evidence_stats: dict[str, Any] = Field(default_factory=dict)


class ReviewEvent(BaseEvent):
    """Event emitted when a review is submitted."""

    event_type: ObservabilityEventType = ObservabilityEventType.REVIEW_DECISION
    episode_id: str
    decision: str
    comments: list[str] = Field(default_factory=list)


# =============================================================================
# WP3 Electronics Events
# =============================================================================


class CircuitValidationEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.CIRCUIT_VALIDATION
    result: bool
    total_draw_a: float
    errors: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


class WireRoutingEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.WIRE_ROUTING
    wire_count: int
    total_length_mm: float
    clearance_passed: bool
    errors: list[str] = Field(default_factory=list)


class PowerBudgetCheckEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.POWER_BUDGET_CHECK
    total_draw_a: float
    max_capacity_a: float
    margin_a: float
    is_safe: bool


class ElectricalFailureEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.ELECTRICAL_FAILURE
    failure_type: str  # e.g., "short_circuit", "overcurrent", "wire_torn"
    component_id: str | None = None
    message: str


class ElecAgentHandoverEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.ELEC_AGENT_HANDOVER
    from_agent: str
    to_agent: str
    iteration_count: int


class CircuitSimulationEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.CIRCUIT_SIMULATION
    duration_s: float
    motor_states: dict[str, str]  # motor_id -> "on"/"off"


# =============================================================================
# WP2 Fluids & Deformable Materials Events
# =============================================================================


class SimulationBackendSelectedEvent(BaseEvent):
    event_type: ObservabilityEventType = (
        ObservabilityEventType.SIMULATION_BACKEND_SELECTED
    )
    backend: str
    fem_enabled: bool
    compute_target: str


class PartBreakageEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.PART_BREAKAGE
    part_label: str
    stress_mpa: float
    ultimate_mpa: float
    location: tuple[float, float, float]
    step: int


class FluidContainmentCheckEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.FLUID_CONTAINMENT_CHECK
    fluid_id: str
    ratio: float
    threshold: float
    passed: bool


class FlowRateCheckEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.FLOW_RATE_CHECK
    fluid_id: str
    measured_rate: float
    target_rate: float
    passed: bool


class StressSummaryEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.STRESS_SUMMARY
    part_label: str
    max_von_mises: float
    safety_factor: float


class MeshingFailureEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.MESHING_FAILURE
    part_label: str
    error: str
    retry_count: int
    repaired: bool


class PhysicsInstabilityEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.PHYSICS_INSTABILITY
    kinetic_energy: float
    threshold: float
    step: int


class GpuOomRetryEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.GPU_OOM_RETRY
    original_particles: int
    reduced_particles: int
