from datetime import UTC, datetime
from enum import StrEnum
from typing import Any, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator


class ObservabilityEventType(StrEnum):
    COMPONENT_USAGE = "component_usage"
    TOOL_INVOCATION = "tool_invocation"
    MANUFACTURABILITY_CHECK = "manufacturability_check"
    SCENE_VALIDATION = "scene_validation"
    RENDER_REQUEST = "render_request"
    SIMULATION_REQUEST = "simulation_request"
    SIMULATION_RESULT = "simulation_result"
    COTS_SEARCH = "cots_search"
    PLAN_SUBMISSION = "plan_submission"
    ESCALATION_REQUEST = "escalation_request"
    ESCALATION_DECISION = "escalation_decision"
    LINT_FAILURE = "lint_failure"
    LOGIC_FAILURE = "logic_failure"
    SKILL_EDIT = "skill_edit"


class SimulationFailureReason(StrEnum):
    TIMEOUT = "timeout"
    OUT_OF_BOUNDS = "out_of_bounds"
    FORBID_ZONE_HIT = "forbid_zone_hit"
    PART_BREAKAGE = "part_breakage"
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


class RenderRequestEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.RENDER_REQUEST
    source: str  # "engineer" or "benchmark"
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


class PlanSubmissionEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.PLAN_SUBMISSION
    source: str  # "engineer" or "benchmark"
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


class LintFailureEvent(BaseEvent):
    event_type: ObservabilityEventType = ObservabilityEventType.LINT_FAILURE
    file_type: str  # "code", "markdown", "yaml"
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
