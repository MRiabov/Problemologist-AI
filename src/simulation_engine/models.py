from pydantic import BaseModel, ConfigDict, Field
from typing import Any


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
