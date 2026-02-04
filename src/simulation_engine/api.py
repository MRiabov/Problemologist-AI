from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field

from src.compiler.models import SimResult


class SimulationRequest(BaseModel):
    """
    Input schema for the simulation service.
    """

    mjcf_xml: str = Field(..., description="The MJCF XML content for the simulation.")
    agent_script: str = Field(default="", description="Optional Python control logic.")
    duration: float = Field(
        default=5.0, description="Simulation duration in logic seconds."
    )
    goal_pos: tuple[float, float, float] | None = Field(
        None, description="Optional success zone (x, y, z)."
    )
    config: dict[str, Any] = Field(
        default_factory=dict, description="Additional configuration parameters."
    )


class SimulationOutcome(StrEnum):
    SUCCESS = "success"
    TIMEOUT = "timeout"
    CRASH = "crash"
    ERROR = "error"


class SimulationErrorType(StrEnum):
    TIMEOUT_ERROR = "TimeoutError"
    CRASH_ERROR = "CrashError"
    RUNTIME_ERROR = "RuntimeError"
    UNKNOWN_ERROR = "UnknownError"


class SimulationResponse(BaseModel):
    """
    Output schema for the simulation service.
    """

    success: bool
    outcome: SimulationOutcome = Field(
        ..., description="Status string: success, timeout, crash, or error."
    )
    result: SimResult | None = Field(
        None, description="Detailed metrics if successful."
    )
    error: str | None = Field(None, description="Error message if failed.")
    error_type: SimulationErrorType | None = Field(
        None, description="The type of error: TimeoutError, CrashError, etc."
    )
