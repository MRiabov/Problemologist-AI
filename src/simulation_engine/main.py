import tempfile
import os
from fastapi import FastAPI, HTTPException
from .api import SimulationRequest, SimulationResponse
from .runner import run_isolated

app = FastAPI(title="Problemologist Simulation Engine")


@app.post("/simulate", response_model=SimulationResponse)
async def simulate(request: SimulationRequest):
    """
    Runs a MuJoCo simulation isolated in a separate process.
    """
    # Create a temporary file for the MJCF model
    with tempfile.NamedTemporaryFile(suffix=".xml", mode="w", delete=False) as tmp:
        tmp.write(request.model_xml)
        tmp_path = tmp.name

    try:
        result = run_isolated(
            model_path=tmp_path,
            agent_script=request.agent_script,
            max_steps=request.max_steps,
            timeout=request.timeout
        )
        return SimulationResponse(**result)
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)


@app.get("/health")
async def health():
    return {"status": "ok"}
