import uuid

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from controller.agent.benchmark.graph import run_generation_session
from shared.enums import ResponseStatus

router = APIRouter(prefix="/simulation", tags=["simulation"])


class RunSimulationRequest(BaseModel):
    session_id: str
    compound_json: str = "{}"


@router.post("/run", status_code=202)
async def run_simulation(request: RunSimulationRequest):
    """
    Trigger a benchmark generation session (simulation pipeline).
    This endpoint reuses the benchmark generation graph but is called 'simulation'
    in the frontend for historical reasons.
    """
    try:
        # Convert string ID to UUID if possible, or use as is if string allowed?
        # The backend expects UUID. The frontend generates a string like "sim-..."
        # We might need to generate a proper UUID and map it, or change frontend to send UUID.
        # But wait, run_generation_session expects UUID | None.
        # If we pass a non-UUID string it might fail validation in DB model if type is UUID.

        # Let's check model. Episode.id is UUID.
        # Frontend sends "sim-randomstring".
        # We should probably generate a new UUID here and ignore the frontend session ID
        # OR force frontend to send UUID.
        # Since I can't easily change frontend logic to UUID without breaking "sim-" convention elsewhere?
        # Actually, let's look at frontend client.ts again.

        # Ideally, we return a new UUID and frontend uses that.
        # But frontend sets sessionId locally.

        # Let's try to parse it. If it fails, generate new UUID.
        # But wait, frontend redirects to /benchmark/SESSION_ID?
        # No, fetchEpisodes() reloads the list.

        # So we can just generate a valid UUID here and return it.
        # The frontend uses the passed sessionId for... logs?

        # The frontend generates a 'sim-' string which is not a valid UUID.
        # We must generate a real UUID for the database primary key.
        # We can store the frontend's session_id in metadata if we want to trace it back,
        # but for now we just start a fresh session.
        real_session_id = uuid.uuid4()

        prompt = "Generate a benchmark simulation"

        import asyncio

        asyncio.create_task(
            run_generation_session(prompt=prompt, session_id=real_session_id)
        )

        return {
            "status": ResponseStatus.ACCEPTED,
            "message": "Simulation started",
            "session_id": str(real_session_id),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
