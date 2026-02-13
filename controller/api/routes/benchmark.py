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
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None

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

    custom_objectives = {}
    if request.max_cost is not None:
        custom_objectives["max_unit_cost"] = request.max_cost
    if request.max_weight is not None:
        custom_objectives["max_weight"] = request.max_weight
    if request.target_quantity is not None:
        custom_objectives["target_quantity"] = request.target_quantity

    # Run the generation in the background
    background_tasks.add_task(
        run_generation_session,
        request.prompt,
        session_id=session_id,
        custom_objectives=custom_objectives,
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


class UpdateObjectivesRequest(BaseModel):
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None


@router.post("/{session_id}/objectives")
async def update_objectives(
    session_id: uuid.UUID,
    request: UpdateObjectivesRequest,
    db: AsyncSession = Depends(get_db),
):
    """
    Update objectives.yaml for a specific session.
    """
    import os
    import httpx
    import yaml
    from controller.clients.worker import WorkerClient

    # Verify session exists
    stmt = select(GenerationSessionModel).where(
        GenerationSessionModel.session_id == session_id
    )
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_url, session_id=str(session_id), http_client=http_client
        )

        try:
            # Read existing
            content = await client.read_file("objectives.yaml")
            obj_data = yaml.safe_load(content) or {}

            if "constraints" not in obj_data:
                obj_data["constraints"] = {}

            if request.max_cost is not None:
                obj_data["constraints"]["max_unit_cost"] = request.max_cost
            if request.max_weight is not None:
                obj_data["constraints"]["max_weight"] = request.max_weight
            if request.target_quantity is not None:
                obj_data["constraints"]["target_quantity"] = request.target_quantity

            # Write back
            new_content = yaml.dump(obj_data, sort_keys=False)
            await client.write_file("objectives.yaml", new_content)

            return {
                "status": ResponseStatus.SUCCESS,
                "message": "Objectives updated",
                "objectives": obj_data["constraints"],
            }

        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Failed to update objectives: {e}"
            )
