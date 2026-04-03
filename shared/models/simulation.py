from enum import StrEnum
from typing import Any, Literal

from pydantic import BaseModel, Field

from shared.enums import FailureReason
from shared.simulation.schemas import SimulatorBackendType


class RenderMode(StrEnum):
    STATIC_PREVIEW = "static_preview"
    SIMULATION_VIDEO = "simulation_video"


class RendererCapabilities(BaseModel):
    backend_name: str
    artifact_modes_supported: list[RenderMode] = Field(default_factory=list)
    supports_default_view: bool = True
    supports_named_cameras: bool = True
    supports_rgb: bool = True
    supports_depth: bool = False
    supports_segmentation: bool = False
    default_view_label: str | None = None


class SimulationRenderProvenance(BaseModel):
    artifact_mode: RenderMode = RenderMode.SIMULATION_VIDEO
    backend_type: SimulatorBackendType
    backend_name: str
    renderer_capabilities: RendererCapabilities | None = None
    available_camera_names: list[str] = Field(default_factory=list)
    camera_candidates: list[str] = Field(default_factory=list)
    resolved_camera_name: str | None = None
    used_default_view: bool = False
    resolved_default_view_label: str | None = None
    capture_interval_seconds: float | None = None
    capture_interval_steps: int | None = None
    captured_frame_count: int = 0
    render_error: str | None = None


class SimulationFailure(BaseModel):
    """Structured failure information for simulation."""

    reason: FailureReason
    detail: str | None = None

    def __str__(self) -> str:
        if self.detail:
            return f"{self.reason}:{self.detail}"
        return str(self.reason)

    def __eq__(self, other: Any) -> bool:
        """
        Supports exact match with another SimulationFailure or
        a reason-only match with a FailureReason enum.
        """
        if isinstance(other, FailureReason):
            return self.reason == other
        return super().__eq__(other)


class StressSummary(BaseModel):
    part_label: str
    max_von_mises_pa: float
    mean_von_mises_pa: float
    safety_factor: float  # ultimate_stress / max_von_mises
    location_of_max: tuple[float, float, float]
    utilization_pct: float  # max_stress / yield_stress * 100


class FluidMetricResult(BaseModel):
    metric_type: str  # "fluid_containment" | "flow_rate"
    fluid_id: str
    measured_value: float
    target_value: float
    passed: bool


class StressFieldData(BaseModel):
    """Serializable version of StressField."""

    nodes: list[list[float]]  # (N, 3)
    stress: list[float]  # (N,) von Mises stress


class SimulationMetrics(BaseModel):
    total_time: float = 0.0
    total_energy: float = 0.0
    max_velocity: float = 0.0
    max_stress: float = 0.0
    success: bool = False
    fail_reason: str | None = None
    fail_mode: FailureReason | None = None
    failure: SimulationFailure | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    stress_fields: dict[str, StressFieldData] = Field(
        default_factory=dict
    )  # part_label -> StressFieldData
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    events: list[dict] = Field(default_factory=list)
    confidence: Literal["low", "medium", "high", "approximate"] = "high"


class SimulationResult(BaseModel):
    success: bool
    summary: str
    failure_reason: str | None = None
    fail_mode: FailureReason | None = None
    failure: SimulationFailure | None = None
    render_provenance: SimulationRenderProvenance | None = None
    render_paths: list[str] = Field(default_factory=list)
    render_object_store_keys: dict[str, str] = Field(default_factory=dict)
    mjcf_content: str | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    stress_fields: dict[str, StressFieldData] = Field(default_factory=dict)
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    total_cost: float = 0.0
    total_weight_g: float = 0.0
    confidence: Literal["low", "medium", "high", "approximate"] = "high"


class MultiRunResult(BaseModel):
    """Result of batched runtime-randomization verification."""

    num_scenes: int
    success_count: int
    success_rate: float
    is_consistent: bool  # True if all runs agree on success/fail
    individual_results: list[SimulationMetrics]
    fail_reasons: list[str]  # Unique failure reasons across runs
    scene_build_count: int = 1
    backend_run_count: int = 1
    batched_execution: bool = True

    model_config = {"extra": "forbid"}
