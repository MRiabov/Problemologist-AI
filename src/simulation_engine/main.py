from fastapi import FastAPI, HTTPException

from src.simulation_engine.api import SimulationRequest, SimulationResponse
from src.simulation_engine.runner import run_isolated

app = FastAPI(title="Problemologist Simulation Service")


@app.post("/simulate", response_model=SimulationResponse)
async def simulate(request: SimulationRequest):
    """
    Triggers an isolated MuJoCo simulation.
    """
    try:
        # We run the isolated process.
        # Since run_isolated uses multiprocessing, it might block the event loop
        # if not called in a thread/executor, but for simple MVP it's often fine.
        # In a high-load scenario, we'd use an async executor or a task queue like Celery/Huey.
        result_data = run_isolated(
            xml_string=request.mjcf_xml,
            duration=request.duration,
            timeout=request.config.get("timeout", 30.0),
            agent_script=request.agent_script,
            goal_pos=request.goal_pos,
        )

        if result_data["success"]:
            return SimulationResponse(
                success=True, outcome="success", result=result_data["result"]
            )
        return SimulationResponse(
            success=False,
            outcome=result_data.get("error_type", "error").lower(),
            error=result_data.get("message"),
            error_type=result_data.get("error_type"),
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
async def health():
    return {"status": "ok"}
