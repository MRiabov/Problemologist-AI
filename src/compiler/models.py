from datetime import datetime
from typing import Any

from pydantic import BaseModel, ConfigDict, Field


class CostBreakdown(BaseModel):
    model_config = ConfigDict(extra="ignore")

    process: str  # enum?
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float = 0.0
    is_reused: bool = False
    details: dict[str, Any] = Field(default_factory=dict)
    pricing_explanation: str = ""


class ValidationViolation(BaseModel):
    description: str
    severity: str = "error"  # error, warning


class ValidationReport(BaseModel):
    model_config = ConfigDict(extra="ignore")

    status: str  # "pass", "fail"
    manufacturability_score: float
    violations: list[ValidationViolation]
    cost_analysis: CostBreakdown
    parts: list[dict[str, Any]] = Field(default_factory=list)
    stl_path: str | None = None
    error: str | None = None


class Observation(BaseModel):
    step: int
    time: float
    state_vector: list[float]
    energy_consumed: float
    damage_detected: float


class SimResult(BaseModel):
    model_config = ConfigDict(extra="ignore")

    success: bool
    total_energy: float
    total_damage: float
    observations: list[Observation]
    metadata: dict[str, Any] = Field(default_factory=dict)

    # Note: No need for from_dict anymore, use model_validate() or model_validate_json()


class EpisodeSummary(BaseModel):
    id: str
    timestamp: datetime
    name: str


class DashStep(BaseModel):
    index: int
    type: str
    agent_role: str | None
    content: str | None
    tool_name: str | None
    tool_input: str | None
    tool_output: str | None
    metadata: dict[str, Any]
    artifacts: list[str] = Field(default_factory=list)


class DashEpisode(BaseModel):
    id: str
    name: str
    steps: list[DashStep]
