from typing import Any

from pydantic import BaseModel, Field

from shared.enums import FailureReason as SimulationFailureMode


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
    fail_mode: SimulationFailureMode | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    stress_fields: dict[str, StressFieldData] = Field(
        default_factory=dict
    )  # part_label -> StressFieldData
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    events: list[dict] = Field(default_factory=list)
    confidence: str = "high"


class SimulationResult(BaseModel):
    success: bool
    summary: str
    failure_reason: str | None = None
    fail_mode: SimulationFailureMode | None = None
    render_paths: list[str] = Field(default_factory=list)
    mjcf_content: str | None = None
    stress_summaries: list[StressSummary] = Field(default_factory=list)
    stress_fields: dict[str, StressFieldData] = Field(default_factory=dict)
    fluid_metrics: list[FluidMetricResult] = Field(default_factory=list)
    total_cost: float = 0.0
    total_weight_g: float = 0.0
    confidence: str = "high"
