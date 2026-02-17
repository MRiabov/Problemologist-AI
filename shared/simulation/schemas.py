from enum import Enum
from typing import Any

from pydantic import BaseModel


from shared.enums import SimulationFailureMode


class SimulatorBackendType(str, Enum):
    MUJOCO = "mujoco"  # Rigid-body only, fast, no FEM/fluids
    GENESIS = "genesis"  # FEM + MPM fluids, requires more compute


class SimulationRequest(BaseModel):
    session_id: str
    mjcf_content: str | None = None
    compound_json: str | None = None
    backend: SimulatorBackendType = SimulatorBackendType.GENESIS
    dt: float = 0.01
    duration: float = 5.0
    randomize: bool = False


class BackendSimulationResult(BaseModel):
    success: bool
    failure_reason: SimulationFailureMode | None = None
    logs: list[str] = []
    metrics: dict[str, Any] = {}


class CustomObjectives(BaseModel):
    """User-defined objectives/constraints for the benchmark."""

    max_unit_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None
    additional_constraints: dict[str, Any] = {}


class RandomizationStrategy(BaseModel):
    """Structured strategy for benchmark randomization."""

    theme: str
    target_object_properties: dict[str, Any] = {}
    environment_perturbations: dict[str, Any] = {}
    difficulty_score: float = 0.5
    reasoning: str | None = None


class ValidationResult(BaseModel):
    """Result of a benchmark validation/simulation run."""

    valid: bool
    cost: float = 0.0
    logs: list[str] = []
    render_paths: list[str] = []
    render_data: list[bytes] | None = None


class AssetMetadata(BaseModel):
    """Metadata for a benchmark asset."""

    theme: str | None = None
    complexity: str | None = None
    tags: list[str] = []
    additional_info: dict[str, Any] = {}
