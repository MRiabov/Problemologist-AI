from pydantic import BaseModel, Field
from typing import Any, Optional, List, Dict
import numpy as np
from shared.simulation.schemas import SimulationFailureMode


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


class SimulationMetrics(BaseModel):
    total_time: float
    total_energy: float
    max_velocity: float
    success: bool
    fail_reason: Optional[str] = None
    fail_mode: Optional[SimulationFailureMode] = None
    stress_summaries: List[StressSummary] = Field(default_factory=list)
    stress_fields: Dict[str, Dict] = Field(
        default_factory=dict
    )  # part_label -> {"nodes": ..., "stress": ...}
    fluid_metrics: List[FluidMetricResult] = Field(default_factory=list)
    confidence: str = "high"


class SimulationResult(BaseModel):
    success: bool
    summary: str
    failure_reason: Optional[str] = None
    fail_mode: Optional[SimulationFailureMode] = None
    render_paths: List[str] = Field(default_factory=list)
    mjcf_content: Optional[str] = None
    stress_summaries: List[StressSummary] = Field(default_factory=list)
    stress_fields: Dict[str, Any] = Field(default_factory=dict)
    fluid_metrics: List[FluidMetricResult] = Field(default_factory=list)
    total_cost: float = 0.0
    total_weight_g: float = 0.0
    confidence: str = "high"
