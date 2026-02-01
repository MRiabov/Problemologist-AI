from pydantic import BaseModel
from typing import Dict, Any, Optional


class SimulationRequest(BaseModel):
    """
    Request to run a simulation.
    """
    model_xml: str  # MJCF XML content
    agent_script: str
    max_steps: int = 1000
    timeout: float = 30.0
    config: Dict[str, Any] = {}


class SimulationResponse(BaseModel):
    """
    Response from a simulation run.
    """
    status: str  # SUCCESS, TIMEOUT, CRASH, ERROR
    message: Optional[str] = None
    metrics: Optional[Dict[str, Any]] = None
    steps: Optional[int] = None
