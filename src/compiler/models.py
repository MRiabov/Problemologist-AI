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
    qpos: np.ndarray
    qvel: np.ndarray
    time: float
    sensordata: np.ndarray
    site_xpos: dict[str, np.ndarray]


@dataclass
class SimResult:
    duration: float
    energy: float
    success: bool
    damage: float
    replay_data: list[dict[str, Any]] | None = None
