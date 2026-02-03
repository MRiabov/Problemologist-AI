from datetime import datetime
from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class CostBreakdown:
    process: str
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float = 0.0
    is_reused: bool = False
    details: dict[str, Any] = field(default_factory=dict)
    pricing_explanation: str = ""


@dataclass
class ValidationViolation:
    description: str
    severity: str = "error"  # error, warning


@dataclass
class ValidationReport:
    status: str  # "pass", "fail"
    manufacturability_score: float
    violations: list[ValidationViolation]
    cost_analysis: CostBreakdown
    parts: list[dict[str, Any]] = field(default_factory=list)
    stl_path: str | None = None
    error: str | None = None


@dataclass
class Observation:
    step: int
    time: float
    state_vector: np.ndarray
    energy_consumed: float
    damage_detected: float


@dataclass
class SimResult:
    success: bool
    total_energy: float
    total_damage: float
    observations: list[Observation]
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class EpisodeSummary:
    id: str
    timestamp: datetime
    name: str


@dataclass
class DashStep:
    index: int
    type: str
    agent_role: str | None
    content: str | None
    tool_name: str | None
    tool_input: str | None
    tool_output: str | None
    metadata: dict[str, Any]
    artifacts: list[str] = field(default_factory=list)


@dataclass
class DashEpisode:
    id: str
    name: str
    steps: list[DashStep]
