from typing import Dict, Any, Optional
from pydantic import BaseModel, Field


class SimulationRequest(BaseModel):
    """
    Input schema for the simulation service.
    """
    mjcf_xml: str = Field(..., description="The MJCF XML content for the simulation.")
    agent_script: str = Field(default="", description="Optional Python control logic.")
    duration: float = Field(default=5.0, description="Simulation duration in logic seconds.")
    goal_pos: Optional[tuple[float, float, float]] = Field(None, description="Optional success zone (x, y, z).")
    config: Dict[str, Any] = Field(default_factory=dict, description="Additional configuration parameters.")


class SimulationResponse(BaseModel):
    """
    Output schema for the simulation service.
    """
    success: bool
    outcome: str = Field(..., description="Status string: success, timeout, crash, or error.")
    result: Optional[Dict[str, Any]] = Field(None, description="Detailed metrics if successful.")
    error: Optional[str] = Field(None, description="Error message if failed.")
    error_type: Optional[str] = Field(None, description="The type of error: TimeoutError, CrashError, etc.")