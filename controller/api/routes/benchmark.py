import uuid

from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from pydantic import BaseModel, field_validator
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from controller.agent.benchmark.graph import run_generation_session
from controller.agent.benchmark.schema import GenerationSessionModel
from controller.persistence.db import get_db
from shared.enums import ResponseStatus

router = APIRouter(prefix="/benchmark", tags=["benchmark"])


class BenchmarkGenerateRequest(BaseModel):
    prompt: str

    @field_validator("prompt")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


@router.post("/generate")
async def generate_benchmark(
    request: BenchmarkGenerateRequest, background_tasks: BackgroundTasks
):
    """
    Trigger a new benchmark generation session.
    """
    # We use the graph's run_generation_session which handles its own persistence
    # but we wrap it in a background task to return immediately.
    session_id = uuid.uuid4()

    # Run the generation in the background
    background_tasks.add_task(
        run_generation_session, request.prompt, session_id=session_id
    )

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
async def get_session(session_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    stmt = select(GenerationSessionModel).where(
        GenerationSessionModel.session_id == session_id
    )
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    return session