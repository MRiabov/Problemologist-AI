import os
import uuid
from typing import Annotated

import httpx
import structlog
import yaml
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field, field_validator
from sqlalchemy import func, select
from sqlalchemy.ext.asyncio import AsyncSession

from controller.agent.benchmark.graph import run_generation_session
from controller.api.benchmark_tasks import launch_tracked_benchmark_task
from controller.api.schemas import (
    AssetResponse,
    BenchmarkConfirmResponse,
    BenchmarkGenerateResponse,
    BenchmarkObjectivesResponse,
    EpisodeResponse,
    TraceResponse,
)
from controller.clients.worker import WorkerClient
from controller.persistence.db import get_db
from controller.persistence.models import Episode, Trace
from shared.enums import GenerationKind, ResponseStatus
from shared.models.schemas import CustomObjectives
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)

router = APIRouter(prefix="/benchmark", tags=["benchmark"])
logger = structlog.get_logger(__name__)

_PROMPT_LOG_PREVIEW_LIMIT = 500


class BenchmarkGenerateRequest(BaseModel):
    prompt: str
    session_id: uuid.UUID | None = None
    start_node: str | None = None
    max_cost: float | None = None
    max_weight: float | None = None
    target_quantity: int | None = None
    seed_id: str | None = None
    seed_dataset: str | None = None
    generation_kind: GenerationKind | None = None
    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)

    @field_validator("prompt")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


@router.post("/generate", response_model=BenchmarkGenerateResponse)
async def generate_benchmark(request: BenchmarkGenerateRequest):
    """
    Trigger a new benchmark generation session.
    """
    # We use the graph's run_generation_session which handles its own persistence
    # but we wrap it in a background task to return immediately.
    session_id = request.session_id or uuid.uuid4()
    logger.info(
        "benchmark_generate_requested",
        session_id=str(session_id),
        prompt=request.prompt,
        prompt_preview=request.prompt[:_PROMPT_LOG_PREVIEW_LIMIT],
        prompt_length=len(request.prompt),
        max_cost=request.max_cost,
        max_weight=request.max_weight,
        target_quantity=request.target_quantity,
        seed_id=request.seed_id,
        seed_dataset=request.seed_dataset,
        generation_kind=request.generation_kind.value
        if request.generation_kind
        else None,
        start_node=request.start_node,
        backend=request.backend.value,
    )

    custom_objectives = CustomObjectives(
        max_unit_cost=request.max_cost,
        max_weight=request.max_weight,
        target_quantity=request.target_quantity,
    )

    # Run the generation in the background with controller-managed cancellation.
    launch_tracked_benchmark_task(
        session_id,
        run_generation_session(
            request.prompt,
            session_id=session_id,
            custom_objectives=custom_objectives,
            seed_id=request.seed_id,
            seed_dataset=request.seed_dataset,
            generation_kind=request.generation_kind,
            backend=request.backend,
            start_node=request.start_node,
        ),
    )

    return BenchmarkGenerateResponse(
        status=ResponseStatus.ACCEPTED,
        message="Benchmark generation started",
        session_id=session_id,
        episode_id=session_id,
    )


class ConfirmRequest(BaseModel):
    comment: str | None = None
    additional_turns: int = Field(
        default=0,
        ge=0,
        description="Optional extra turns to grant before resuming execution.",
    )


@router.post("/{session_id}/confirm", response_model=BenchmarkConfirmResponse)
async def confirm_benchmark(
    session_id: uuid.UUID,
    request: ConfirmRequest,
    db: Annotated[AsyncSession, Depends(get_db)],
):
    """
    Confirm and continue benchmark generation after planning.
    """
    from controller.agent.benchmark.graph import continue_generation_session
    from controller.persistence.models import Trace
    from shared.enums import TraceType

    # Record user comment if provided
    logger.info(
        "benchmark_confirm_requested",
        session_id=str(session_id),
        has_comment=bool(request.comment),
        comment=request.comment,
        additional_turns=request.additional_turns,
    )

    # Record user comment if provided
    if request.comment:
        from shared.models.schemas import TraceMetadata

        metadata = TraceMetadata(additional_info={"comment": request.comment})
        trace_record = Trace(
            episode_id=session_id,
            trace_type=TraceType.LOG,
            content=f"User confirmation comment: {request.comment}",
            name="user_confirmation",
            metadata_vars=metadata.model_dump(),
        )
        db.add(trace_record)
        await db.commit()

    if request.additional_turns > 0:
        from controller.agent.execution_limits import grant_episode_additional_turns

        await grant_episode_additional_turns(
            episode_id=session_id,
            additional_turns=request.additional_turns,
        )

    launch_tracked_benchmark_task(
        session_id,
        continue_generation_session(
            session_id=session_id,
        ),
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
    session_id: uuid.UUID,
    db: Annotated[AsyncSession, Depends(get_db)],
    include_traces: bool = True,
    trace_limit: int = 200,
    trace_content_limit: int = 4000,
):
    from sqlalchemy.orm import selectinload

    stmt = (
        select(Episode)
        .where(Episode.id == session_id)
        .options(selectinload(Episode.assets))
    )
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    last_trace_id = await db.scalar(
        select(func.max(Trace.id)).where(Trace.episode_id == session_id)
    )

    response = EpisodeResponse(
        id=session.id,
        user_session_id=session.user_session_id,
        task=session.task,
        status=session.status,
        created_at=session.created_at,
        updated_at=session.updated_at,
        skill_git_hash=session.skill_git_hash,
        template_versions=session.template_versions,
        metadata_vars=session.metadata_vars,
        todo_list=session.todo_list,
        journal=session.journal,
        plan=session.plan,
        validation_logs=session.validation_logs,
        last_trace_id=last_trace_id,
        traces=[],
        assets=[AssetResponse.model_validate(a) for a in session.assets],
    )

    if not include_traces:
        return response

    safe_trace_limit = max(1, min(trace_limit, 500))
    safe_content_limit = max(200, min(trace_content_limit, 20000))

    trace_stmt = (
        select(Trace)
        .where(Trace.episode_id == session_id)
        .order_by(Trace.id.desc())
        .limit(safe_trace_limit)
    )
    trace_rows = (await db.execute(trace_stmt)).scalars().all()
    trace_rows = list(reversed(trace_rows))

    trace_responses: list[TraceResponse] = []
    for trace in trace_rows:
        tr = TraceResponse.model_validate(trace)
        if tr.content and len(tr.content) > safe_content_limit:
            tr.content = tr.content[:safe_content_limit] + "\n...[truncated]"
        trace_responses.append(tr)

    response.traces = trace_responses
    return response


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
    Update benchmark_definition.yaml for a specific session.
    """
    # The original code had these imports inside the function.
    # They are now moved to the top-level imports.

    # Verify session exists
    stmt = select(Episode).where(Episode.id == session_id)
    result = await db.execute(stmt)
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    logger.info(
        "benchmark_objectives_update_requested",
        session_id=str(session_id),
        max_cost=request.max_cost,
        max_weight=request.max_weight,
        target_quantity=request.target_quantity,
    )

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
            from worker_heavy.utils.file_validation import (
                validate_benchmark_definition_yaml,
            )

            # Read existing
            content = await client.read_file("benchmark_definition.yaml")
            is_valid, benchmark_result = validate_benchmark_definition_yaml(
                content,
                session_id=str(session_id),
            )
            if not is_valid:
                raise ValueError("; ".join(benchmark_result))
            obj_data = benchmark_result

            if request.max_cost is not None:
                obj_data.constraints.max_unit_cost = request.max_cost
                obj_data.constraints.estimated_solution_cost_usd = (
                    request.max_cost / 1.5
                )
            if request.max_weight is not None:
                obj_data.constraints.max_weight_g = request.max_weight
                obj_data.constraints.estimated_solution_weight_g = (
                    request.max_weight / 1.5
                )
            if request.target_quantity is not None:
                obj_data.constraints.target_quantity = request.target_quantity

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
            new_content = yaml.dump(obj_data.model_dump(mode="json"), sort_keys=False)
            await client.write_file("benchmark_definition.yaml", new_content)

            return BenchmarkObjectivesResponse(
                status=ResponseStatus.SUCCESS,
                message="Objectives updated",
                objectives=obj_data.constraints.model_dump(),
            )

        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Failed to update objectives: {e}"
            ) from e
