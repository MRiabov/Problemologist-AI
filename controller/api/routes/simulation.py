import uuid

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from controller.agent.benchmark.graph import run_generation_session
from shared.enums import ResponseStatus
from shared.simulation.schemas import SimulatorBackendType

router = APIRouter(prefix="/simulation", tags=["simulation"])


class RunSimulationRequest(BaseModel):
    session_id: str
    compound_json: str = "{}"
    script_content: str | None = None
    backend: SimulatorBackendType = SimulatorBackendType.GENESIS


@router.post("/run", status_code=202)
async def run_simulation(request: RunSimulationRequest):
    """
    Trigger a benchmark generation session (simulation pipeline).
    This endpoint reuses the benchmark generation graph but is called 'simulation'
    in the frontend for historical reasons.
    """
    try:
        # The frontend generates a 'sim-' string which is not a valid UUID.
        # We must generate a real UUID for the database primary key.
        real_session_id = uuid.uuid4()

        prompt = "Generate a benchmark simulation"
        script_to_run = request.script_content

        # Fallback to compound_json if it looks like a script (legacy/frontend quirk)
        if not script_to_run and request.compound_json:
            potential_script = request.compound_json.strip()
            if (
                potential_script.startswith("import")
                or "def build" in potential_script
                or "from build123d" in potential_script
            ):
                script_to_run = potential_script

        if script_to_run:
            prompt = "Simulate provided design"

        import asyncio

        asyncio.create_task(
            run_generation_session(
                prompt=prompt,
                session_id=real_session_id,
                backend=request.backend,
                initial_script=script_to_run,
            )
        )

        return {
            "status": ResponseStatus.ACCEPTED,
            "message": "Simulation started",
            "session_id": str(real_session_id),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
