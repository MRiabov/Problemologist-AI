from enum import Enum
from typing import Any

from pydantic import BaseModel


class SimulatorBackendType(str, Enum):
    MUJOCO = "mujoco"  # Rigid-body only, fast, no FEM/fluids
    GENESIS = "genesis"  # FEM + MPM fluids, requires more compute


class SimulationFailureMode(str, Enum):
    """Specific failure modes for physics and electronics simulation."""

    # WP3 Electronics failures
    SHORT_CIRCUIT = "FAILED_SHORT_CIRCUIT"
    OVERCURRENT_SUPPLY = "FAILED_OVERCURRENT_SUPPLY"
    OVERCURRENT_WIRE = "FAILED_OVERCURRENT_WIRE"
    OPEN_CIRCUIT = "FAILED_OPEN_CIRCUIT"
    WIRE_TORN = "FAILED_WIRE_TORN"

    # WP2 Fluids & Physics failures
    ASSET_GENERATION = "FAILED_ASSET_GENERATION"
    FLUID_OBJECTIVE_FAILED = "FAILED_FLUID_OBJECTIVE"
    PART_BREAKAGE = "FAILED_PART_BREAKAGE"
    STRESS_OBJECTIVE_EXCEEDED = "FAILED_STRESS_OBJECTIVE"
    ELECTRONICS_FLUID_DAMAGE = "FAILED_ELECTRONICS_FLUID_DAMAGE"
    PHYSICS_INSTABILITY = "FAILED_PHYSICS_INSTABILITY"


class SimulationRequest(BaseModel):
    session_id: str
    mjcf_content: str | None = None
    compound_json: str | None = None
    backend: SimulatorBackendType = SimulatorBackendType.MUJOCO
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
    estimated_unit_cost_usd: float | None = None
    estimated_weight_kg: float | None = None


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
