import uuid

from fastapi import APIRouter, BackgroundTasks, HTTPException
from sqlmodel import select

from controller.agent.benchmark.graph import run_generation_session
from controller.agent.benchmark.schema import GenerationSessionModel
from controller.persistence.db import get_db
from shared.enums import ResponseStatus

router = APIRouter(prefix="/benchmark", tags=["benchmark"])


@router.post("/generate")
async def generate_benchmark(prompt: str, background_tasks: BackgroundTasks):
    """
    Trigger a new benchmark generation session.
    """
    # We use the graph's run_generation_session which handles its own persistence
    # but we wrap it in a background task to return immediately.
    session_id = uuid.uuid4()

    # Run the generation in the background
    background_tasks.add_task(run_generation_session, prompt, session_id=session_id)

    return {
        "status": ResponseStatus.ACCEPTED,
        "message": "Benchmark generation started",
        "session_id": session_id,
    }


# Note: We might need an endpoint to get the status/result of a specific session
# but run_generation_session returns the final state, so for now we rely on
# direct DB access or existing episode endpoints if we wrapped it in an episode.
# However, run_generation_session currently creates its own GenerationSessionModel entry.
# We should probably expose a getter for that.


@router.get("/{session_id}")
async def get_benchmark_session(session_id: uuid.UUID):
    async with get_db() as db:
        stmt = select(GenerationSessionModel).where(
            GenerationSessionModel.session_id == session_id
        )
        result = await db.execute(stmt)
        session = result.scalar_one_or_none()

        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return session
