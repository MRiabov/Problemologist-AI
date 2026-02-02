from typing import Any

from pydantic import BaseModel, Field


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


class SimulationResponse(BaseModel):
    """
    Output schema for the simulation service.
    """

    success: bool
    outcome: str = Field(
        ..., description="Status string: success, timeout, crash, or error."
    )
    result: dict[str, Any] | None = Field(
        None, description="Detailed metrics if successful."
    )
    error: str | None = Field(None, description="Error message if failed.")
    error_type: str | None = Field(
        None, description="The type of error: TimeoutError, CrashError, etc."
    )
