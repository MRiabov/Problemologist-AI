import os
import uuid
from typing import Annotated

import httpx
import yaml
from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from pydantic import BaseModel, field_validator
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession

from controller.agent.benchmark.graph import run_generation_session
from controller.api.schemas import (
    BenchmarkConfirmResponse,
    BenchmarkGenerateResponse,
    BenchmarkObjectivesResponse,
    EpisodeResponse,
)
from controller.clients.worker import WorkerClient
from controller.persistence.db import get_db
from controller.persistence.models import Episode
from shared.enums import ResponseStatus
from shared.simulation.schemas import SimulatorBackendType
from shared.models.schemas import CustomObjectives

router = APIRouter(prefix="/benchmark", tags=["benchmark"])


class BenchmarkGenerateRequest(BaseModel):
    prompt: str
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None
    backend: SimulatorBackendType = SimulatorBackendType.GENESIS

    @field_validator("prompt")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


@router.post("/generate", response_model=BenchmarkGenerateResponse)
async def generate_benchmark(
    request: BenchmarkGenerateRequest, background_tasks: BackgroundTasks
):
    """
    Trigger a new benchmark generation session.
    """
    # We use the graph's run_generation_session which handles its own persistence
    # but we wrap it in a background task to return immediately.
    session_id = uuid.uuid4()

    custom_objectives = CustomObjectives(
        max_unit_cost=request.max_cost,
        max_weight=request.max_weight,
        target_quantity=request.target_quantity,
    )

    # Run the generation in the background
    background_tasks.add_task(
        run_generation_session,
        request.prompt,
        session_id=session_id,
        custom_objectives=custom_objectives,
        backend=request.backend,
    )

    return BenchmarkGenerateResponse(
        status=ResponseStatus.ACCEPTED,
        message="Benchmark generation started",
        session_id=session_id,
    )


class ConfirmRequest(BaseModel):
    comment: str | None = None


@router.post("/{session_id}/confirm", response_model=BenchmarkConfirmResponse)
async def confirm_benchmark(
    session_id: uuid.UUID,
    request: ConfirmRequest,
    background_tasks: BackgroundTasks,
    db: Annotated[AsyncSession, Depends(get_db)],
):
    """
    Confirm and continue benchmark generation after planning.
    """
    from controller.agent.benchmark.graph import continue_generation_session
    from controller.persistence.models import Trace
    from shared.enums import TraceType

    # Record user comment if provided
    if request.comment:
        trace_record = Trace(
            episode_id=session_id,
            trace_type=TraceType.LOG,
            content=f"User confirmation comment: {request.comment}",
            name="user_confirmation",
            metadata_vars={"comment": request.comment},
        )
        db.add(trace_record)
        await db.commit()

    background_tasks.add_task(
        continue_generation_session,
        session_id=session_id,
    )

    return BenchmarkConfirmResponse(
        status=ResponseStatus.ACCEPTED,
        message="Benchmark generation proceeding",
    )


# Note: We might need an endpoint to get the status/result of a specific session
# but run_generation_session returns the final state, so for now we rely on
# direct DB access or existing episode endpoints if we wrapped it in an episode.
# but we wrap it in a background task to return immediately.
# We should probably expose a getter for that.


@router.get("/{session_id}", response_model=EpisodeResponse)
async def get_session(
    session_id: uuid.UUID, db: Annotated[AsyncSession, Depends(get_db)]
):
    from sqlalchemy.orm import selectinload

    stmt = (
        select(Episode)
        .where(Episode.id == session_id)
        .options(selectinload(Episode.traces), selectinload(Episode.assets))
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


@router.post("/{session_id}/objectives", response_model=BenchmarkObjectivesResponse)
async def update_objectives(
    session_id: uuid.UUID,
    request: UpdateObjectivesRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
):
    """
    Update objectives.yaml for a specific session.
    """
    # The original code had these imports inside the function.
    # They are now moved to the top-level imports.

    # Verify session exists
    stmt = select(Episode).where(Episode.id == session_id)
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    from controller.agent.config import settings

    worker_light_url = os.getenv("WORKER_LIGHT_URL", "http://worker-light:8001")
    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_light_url,
            session_id=str(session_id),
            http_client=http_client,
            heavy_url=settings.worker_heavy_url,
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

            # Actually, we need to MERGE metadata_vars to avoid losing objectives.
            # Update the episode's metadata_vars as well to persist it locally
            from shared.models.schemas import CustomObjectives, EpisodeMetadata

            metadata = EpisodeMetadata.model_validate(session.metadata_vars or {})
            if not metadata.custom_objectives:
                metadata.custom_objectives = CustomObjectives()

            if request.max_cost is not None:
                metadata.custom_objectives.max_unit_cost = request.max_cost
            if request.max_weight is not None:
                metadata.custom_objectives.max_weight = request.max_weight
            if request.target_quantity is not None:
                metadata.custom_objectives.target_quantity = request.target_quantity

            session.metadata_vars = metadata.model_dump()

            # Write back
            new_content = yaml.dump(obj_data, sort_keys=False)
            await client.write_file("objectives.yaml", new_content)

            return BenchmarkObjectivesResponse(
                status=ResponseStatus.SUCCESS,
                message="Objectives updated",
                objectives=obj_data["constraints"],
            )

        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Failed to update objectives: {e}"
            ) from e
