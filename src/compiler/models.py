from datetime import datetime
from dataclasses import dataclass, field
from typing import Any


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
    state_vector: list[float]
    energy_consumed: float
    damage_detected: float


@dataclass
    observations: list[Observation]
    metadata: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "SimResult":
        """Reconstructs SimResult from a dictionary (e.g., from JSON output)."""
        observations = [
            Observation(
                step=o["step"],
                time=o["time"],
                state_vector=o["state_vector"],
                energy_consumed=o["energy_consumed"],
                damage_detected=o["damage_detected"],
            )
            for o in data.get("observations", [])
        ]
        return SimResult(
            success=data.get("success", False),
            total_energy=data.get("total_energy", 0.0),
            total_damage=data.get("total_damage", 0.0),
            observations=observations,
            metadata=data.get("metadata", {}),
        )


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
