from enum import Enum
from typing import Any, Optional
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
    mjcf_content: Optional[str] = None
    compound_json: Optional[str] = None
    backend: SimulatorBackendType = SimulatorBackendType.MUJOCO
    dt: float = 0.01
    duration: float = 5.0
    randomize: bool = False


class BackendSimulationResult(BaseModel):
    success: bool
    failure_reason: Optional[SimulationFailureMode] = None
    logs: list[str] = []
    metrics: dict[str, Any] = {}
