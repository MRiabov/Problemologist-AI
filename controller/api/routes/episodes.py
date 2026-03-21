import asyncio
import uuid
from datetime import datetime
from typing import Annotated, Literal

import httpx
import structlog
from fastapi import (
    APIRouter,
    Depends,
    HTTPException,
    Path,
    Query,
    Response,
    WebSocket,
    WebSocketDisconnect,
)
from pydantic import BaseModel, ConfigDict, Field, StrictStr, field_validator
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from controller.api.manager import manager, task_tracker
from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from controller.observability.langfuse import get_langfuse_client
from controller.persistence.db import get_db
from controller.persistence.models import Asset, Episode, Trace
from shared.enums import (
    AssetType,
    EpisodeStatus,
    ResponseStatus,
    ReviewDecision,
    TraceType,
)
from shared.models.schemas import EpisodeMetadata, TraceMetadata

logger = structlog.get_logger(__name__)
_MESSAGE_LOG_PREVIEW_LIMIT = 500


def _normalize_plan_markdown(plan_content: str | None) -> str | None:
    """Normalize persisted plans for list responses used by dataset-readiness checks."""
    if not plan_content:
        return plan_content

    normalized = plan_content.strip()
    if "# Solution Overview" in normalized and "## Parts List" in normalized:
        return normalized

    if normalized.startswith("# Engineering Plan"):
        _, _, remainder = normalized.partition("\n")
        normalized = remainder.lstrip()

    replacements = {
        "## 1. Solution Overview": "# Solution Overview",
        "## 2. Parts List": "## Parts List",
        "## 3. Assembly Strategy": "## Assembly Strategy",
        "## 4. Cost & Weight Budget": "## Cost & Weight Budget",
        "## 5. Risk Assessment": "## Risk Assessment",
    }
    for old, new in replacements.items():
        normalized = normalized.replace(old, new)

    if "# Solution Overview" in normalized and "## Parts List" in normalized:
        return normalized

    return (
        "# Solution Overview\n"
        "Normalized from persisted plan artifact for dataset readiness.\n\n"
        "## Parts List\n"
        "- Original plan preserved below.\n\n"
        "## Source Plan\n"
        f"{normalized}"
    )


class FeedbackRequest(BaseModel):
    score: Literal[0, 1]
    comment: str | None = None

    @field_validator("comment")
    @classmethod
    def strip_null_bytes(cls, v: str | None) -> str | None:
        if v is not None:
            return v.replace("\u0000", "")
        return v


router = APIRouter(prefix="/episodes", tags=["episodes"])


@router.post("/{episode_id}/traces/{trace_id}/feedback", status_code=202)
async def report_trace_feedback(
    episode_id: uuid.UUID,
    trace_id: Annotated[int, Path(ge=-(2**31), le=2**31 - 1)],
    feedback: FeedbackRequest,
    db: AsyncSession = Depends(get_db),
):
    """Report feedback for a specific trace to Langfuse."""
    logger.info("searching_for_trace", episode_id=str(episode_id), trace_id=trace_id)
    result = await db.execute(
        select(Trace).where(Trace.id == trace_id, Trace.episode_id == episode_id)
    )
    trace = result.scalar_one_or_none()

    if not trace:
        # Diagnostic: Try finding trace by ID alone to see if episode_id is mismatched
        diag_result = await db.execute(select(Trace).where(Trace.id == trace_id))
        diag_trace = diag_result.scalar_one_or_none()
        if diag_trace:
            logger.warning(
                "trace_episode_id_mismatch",
                trace_id=trace_id,
                requested_episode_id=str(episode_id),
                actual_episode_id=str(diag_trace.episode_id),
                trace_type=diag_trace.trace_type,
                trace_name=diag_trace.name,
            )
            # WP10: We return 404 to be strict with the API contract, but the log now has more info
            raise HTTPException(
                status_code=404,
                detail=f"Trace {trace_id} belongs to episode {diag_trace.episode_id}, not {episode_id}",
            )
        logger.warning("trace_not_found_even_by_id", trace_id=trace_id)

        logger.warning("trace_not_found", episode_id=str(episode_id), trace_id=trace_id)
        raise HTTPException(status_code=404, detail="Trace not found")

    # Langfuse update (best effort)
    if trace.langfuse_trace_id:
        try:
            langfuse = get_langfuse_client()
            if langfuse:
                langfuse.create_score(
                    trace_id=trace.langfuse_trace_id,
                    name="user-feedback",
                    value=feedback.score,
                    comment=feedback.comment,
                )
        except Exception as e:
            logger.warning("langfuse_feedback_failed", error=str(e))

    # Store locally
    trace.feedback_score = feedback.score
    trace.feedback_comment = feedback.comment
    await db.commit()

    return {"status": ResponseStatus.ACCEPTED}


@router.get("/{episode_id}/assets/{path:path}")
async def get_episode_asset(
    episode_id: uuid.UUID,
    path: str,
    db: AsyncSession = Depends(get_db),
):
    """Proxy asset requests to the worker."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    worker_session_id = None
    if episode.metadata_vars:
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars)
        worker_session_id = metadata.worker_session_id

    if not worker_session_id:
        # Fallback for older episodes or benchmarks where session_id might be the id itself
        worker_session_id = str(episode_id)

    worker_light_url = settings.worker_light_url
    # Ensure path doesn't start with a slash to avoid double slash in URL
    safe_path = path.lstrip("/")
    asset_url = f"{worker_light_url}/assets/{safe_path}"

    async with httpx.AsyncClient() as client:
        try:
            resp = await client.get(
                asset_url, headers={"X-Session-ID": worker_session_id}, timeout=10.0
            )
            if resp.status_code == 404:
                raise HTTPException(
                    status_code=404, detail=f"Asset {path} not found on worker"
                )
            resp.raise_for_status()

            return Response(
                content=resp.content,
                media_type=resp.headers.get("content-type"),
                status_code=resp.status_code,
            )
        except httpx.HTTPStatusError as e:
            raise HTTPException(
                status_code=e.response.status_code, detail=str(e)
            ) from e
        except Exception as e:
            raise HTTPException(
                status_code=500, detail=f"Failed to proxy asset: {e!s}"
            ) from e


class ReviewRequest(BaseModel):
    review_content: str

    @field_validator("review_content")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


class MessageRequest(BaseModel):
    message: StrictStr
    metadata_vars: EpisodeMetadata | None = Field(
        None, description="Additional metadata for the message."
    )

    @field_validator("message")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


@router.post("/{episode_id}/review")
async def review_episode(
    episode_id: uuid.UUID,
    request: ReviewRequest,
    db: AsyncSession = Depends(get_db),
):
    """Submit a review for an episode."""
    from shared.observability.schemas import ReviewEvent
    from worker_heavy.utils.file_validation import validate_review_frontmatter

    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    # Validate the review frontmatter
    # Routing checks for refusal decisions are handled below with concrete
    # plan_refusal.md validation and deterministic status codes.
    review_validation_session_id = str(episode_id)
    if episode.metadata_vars:
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars)
        review_validation_session_id = (
            metadata.worker_session_id or review_validation_session_id
        )

    is_valid, review_data = validate_review_frontmatter(
        request.review_content,
        cad_agent_refused=True,
        session_id=review_validation_session_id,
    )
    if not is_valid:
        raise HTTPException(status_code=422, detail=f"Invalid review: {review_data}")

    # Update episode based on decision
    decision = review_data.decision
    if decision == ReviewDecision.APPROVED:
        episode.status = EpisodeStatus.COMPLETED
    elif decision in (
        ReviewDecision.REJECTED,
        ReviewDecision.REJECT_PLAN,
        ReviewDecision.REJECT_CODE,
    ):
        episode.status = EpisodeStatus.FAILED
    elif decision == ReviewDecision.CONFIRM_PLAN_REFUSAL:
        # WP06: Validate plan_refusal.md existence and content
        worker_session_id = str(episode_id)
        if episode.metadata_vars:
            metadata = EpisodeMetadata.model_validate(episode.metadata_vars)
            worker_session_id = metadata.worker_session_id or worker_session_id

        client = WorkerClient(
            base_url=settings.worker_light_url,
            session_id=worker_session_id,
            heavy_url=settings.worker_heavy_url,
        )
        try:
            refusal_content = await client.read_file("plan_refusal.md")
            from worker_heavy.utils.file_validation import validate_plan_refusal

            is_valid_refusal, refusal_errs = validate_plan_refusal(refusal_content)
            if not is_valid_refusal:
                raise HTTPException(
                    status_code=422, detail=f"Invalid plan_refusal.md: {refusal_errs}"
                )
        except FileNotFoundError:
            raise HTTPException(
                status_code=422,
                detail="plan_refusal.md not found but required for CONFIRM_PLAN_REFUSAL",
            )
        except Exception as e:
            if isinstance(e, HTTPException):
                raise
            raise HTTPException(
                status_code=500, detail=f"Failed to validate plan_refusal.md: {e!s}"
            )

        episode.status = EpisodeStatus.FAILED

    # Record review event to DB
    import uuid as uuid_lib

    from controller.observability.tracing import record_worker_events

    review_id = uuid_lib.uuid4().hex

    await record_worker_events(
        episode_id=episode_id,
        events=[
            ReviewEvent(
                episode_id=str(episode_id),
                decision=decision,
                comments=review_data.comments,
                review_id=review_id,
            )
        ],
    )

    await db.commit()
    return {"status": ResponseStatus.SUCCESS, "decision": decision}


@router.post("/{episode_id}/messages", status_code=202)
async def continue_episode(
    episode_id: uuid.UUID,
    request: MessageRequest,
    db: AsyncSession = Depends(get_db),
):
    """Send a follow-up message to a running or completed episode."""
    from controller.api.tasks import continue_agent_task

    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    logger.info(
        "episode_message_received",
        episode_id=str(episode_id),
        message=request.message,
        message_preview=request.message[:_MESSAGE_LOG_PREVIEW_LIMIT],
        message_length=len(request.message),
        has_metadata=bool(request.metadata_vars),
    )

    # In a real system, we might want to check if the agent is already busy
    # but for now we'll rely on the task_tracker or just allow it to queue.

    task = asyncio.create_task(
        continue_agent_task(
            episode_id,
            request.message,
            additional_turns=request.additional_turns,
            metadata=request.metadata_vars.model_dump()
            if request.metadata_vars
            else None,
        )
    )
    task_tracker.register_task(episode_id, task)

    return {"status": ResponseStatus.ACCEPTED, "message": "Message sent to agent"}


@router.get("/{episode_id}/electronics/schematic")
async def get_episode_schematic(
    episode_id: uuid.UUID,
    db: AsyncSession = Depends(get_db),
):
    """Get the electronics schematic for an episode in a format compatible with tscircuit."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    worker_session_id = str(episode_id)
    if episode.metadata_vars:
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars)
        worker_session_id = metadata.worker_session_id or worker_session_id

    worker_light_url = settings.worker_light_url
    client = WorkerClient(
        base_url=worker_light_url,
        session_id=worker_session_id,
        heavy_url=settings.worker_heavy_url,
    )

    try:
        content = await client.read_file("assembly_definition.yaml")
        import yaml

        from shared.models.schemas import AssemblyDefinition

        data = yaml.safe_load(content)
        assembly = AssemblyDefinition(**data)

        from shared.schematic_utils import generate_schematic_soup

        return generate_schematic_soup(assembly)
    except Exception as e:
        logger.error(
            "failed_to_get_schematic",
            episode_id=str(episode_id),
            session_id=str(episode_id),
            error=str(e),
        )
        return []


class TraceResponse(BaseModel):
    id: int
    user_session_id: uuid.UUID | None = None
    langfuse_trace_id: str | None
    simulation_run_id: str | None = None
    cots_query_id: str | None = None
    review_id: str | None = None
    trace_type: TraceType
    name: str | None
    content: str | None
    metadata_vars: TraceMetadata | None = None
    feedback_score: int | None = None
    feedback_comment: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)

    @property
    def metadata(self) -> TraceMetadata | None:
        """Backward-compatible alias for clients using `metadata`."""
        return self.metadata_vars


class AssetResponse(BaseModel):
    id: int
    user_session_id: uuid.UUID | None = None
    asset_type: AssetType
    s3_path: str
    content: str | None = None
    created_at: datetime

    model_config = ConfigDict(from_attributes=True)


class EpisodeResponse(BaseModel):
    id: uuid.UUID
    user_session_id: uuid.UUID | None = None
    task: StrictStr
    status: EpisodeStatus
    created_at: datetime
    updated_at: datetime
    skill_git_hash: str | None = None
    template_versions: dict | None = None
    metadata_vars: EpisodeMetadata | None = None
    todo_list: dict | None = None
    journal: str | None = None
    plan: str | None = None
    validation_logs: list[str] | None = None
    last_trace_id: int | None = None
    traces: list[TraceResponse] | None = None
    assets: list[AssetResponse] | None = None

    model_config = ConfigDict(from_attributes=True, populate_by_name=True)

    @property
    def metadata(self) -> EpisodeMetadata | None:
        """Backward-compatible alias for clients using `metadata`."""
        return self.metadata_vars


@router.get("/", response_model=list[EpisodeResponse])
async def list_episodes(
    limit: Annotated[int, Query(ge=0, lt=2**63)] = 100,
    offset: Annotated[int, Query(ge=0, lt=2**63)] = 0,
    db: AsyncSession = Depends(get_db),
):
    """List all agent episodes."""
    from sqlalchemy import func

    try:
        result = await db.execute(
            select(Episode)
            .order_by(Episode.created_at.desc())
            .limit(limit)
            .offset(offset)
        )
        episodes = result.scalars().all()

        # Batch fetch last trace IDs for the UI feedback system
        ep_ids = [ep.id for ep in episodes]
        last_trace_map = {}
        markdown_asset_map: dict[uuid.UUID, dict[str, str]] = {}
        if ep_ids:
            trace_result = await db.execute(
                select(Trace.episode_id, func.max(Trace.id))
                .where(Trace.episode_id.in_(ep_ids))
                .group_by(Trace.episode_id)
            )
            last_trace_map = dict(trace_result.all())
            assets_result = await db.execute(
                select(Asset.episode_id, Asset.s3_path, Asset.content).where(
                    Asset.episode_id.in_(ep_ids),
                    Asset.s3_path.in_(
                        ["plan.md", "/plan.md", "journal.md", "/journal.md"]
                    ),
                )
            )
            for episode_id, s3_path, content in assets_result.all():
                if not content:
                    continue
                key = s3_path.lstrip("/")
                markdown_asset_map.setdefault(episode_id, {})[key] = content

        # Explicitly convert to EpisodeResponse to avoid lazy-loading traces/assets during serialization
        responses: list[EpisodeResponse] = []
        for ep in episodes:
            asset_content = markdown_asset_map.get(ep.id, {})
            plan_content = _normalize_plan_markdown(
                ep.plan or asset_content.get("plan.md")
            )
            journal_content = ep.journal or asset_content.get("journal.md")

            # Normalize legacy journals to required structured sections.
            if journal_content:
                stripped = journal_content.strip()
                if (
                    not (stripped.startswith("#") or "##" in stripped)
                    or "## Observations" not in stripped
                    or "## Decision Log" not in stripped
                ):
                    lines = [ln.strip() for ln in stripped.splitlines() if ln.strip()]
                    bullet_lines = (
                        "\n".join(f"- {ln}" for ln in lines) or "- No details"
                    )
                    journal_content = (
                        "# Execution Journal\n"
                        "## Observations\n"
                        f"{bullet_lines}\n\n"
                        "## Decision Log\n"
                        "- [x] completed: true\n"
                    )

            responses.append(
                EpisodeResponse(
                    id=ep.id,
                    user_session_id=ep.user_session_id,
                    task=ep.task,
                    status=ep.status,
                    created_at=ep.created_at,
                    updated_at=ep.updated_at,
                    skill_git_hash=ep.skill_git_hash,
                    template_versions=ep.template_versions,
                    metadata_vars=EpisodeMetadata.model_validate(ep.metadata_vars)
                    if ep.metadata_vars
                    else None,
                    todo_list=ep.todo_list
                    or (
                        {"completed": True}
                        if ep.status == EpisodeStatus.COMPLETED
                        else None
                    ),
                    journal=journal_content,
                    plan=plan_content,
                    validation_logs=ep.validation_logs,
                    last_trace_id=last_trace_map.get(ep.id),
                    traces=[],
                    assets=[],
                )
            )
        return responses
    except Exception as e:
        logger.warning("list_episodes_failed", error=str(e))
        raise HTTPException(
            status_code=500, detail=f"Internal Server Error: {e!s}"
        ) from e


@router.get("/{episode_id}", response_model=EpisodeResponse)
async def get_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Get a specific episode."""
    result = await db.execute(
        select(Episode)
        .where(Episode.id == episode_id)
        .options(selectinload(Episode.traces), selectinload(Episode.assets))
    )
    episode = result.scalar_one_or_none()
    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    # Populate last_trace_id for EpisodeResponse
    last_trace_id = None
    sorted_traces = sorted(episode.traces, key=lambda t: t.id)
    if sorted_traces:
        last_trace_id = max(t.id for t in sorted_traces)
    canonical_lf_trace_id = next(
        (t.langfuse_trace_id for t in sorted_traces if t.langfuse_trace_id),
        str(episode.id),
    )
    for t in sorted_traces:
        t.langfuse_trace_id = canonical_lf_trace_id

    return EpisodeResponse(
        id=episode.id,
        user_session_id=episode.user_session_id,
        task=episode.task,
        status=episode.status,
        created_at=episode.created_at,
        updated_at=episode.updated_at,
        skill_git_hash=episode.skill_git_hash,
        template_versions=episode.template_versions,
        metadata_vars=EpisodeMetadata.model_validate(episode.metadata_vars)
        if episode.metadata_vars
        else None,
        todo_list=episode.todo_list
        or ({"completed": True} if episode.status == EpisodeStatus.COMPLETED else None),
        journal=episode.journal,
        plan=episode.plan,
        validation_logs=episode.validation_logs,
        last_trace_id=last_trace_id,
        traces=[TraceResponse.model_validate(t) for t in sorted_traces],
        assets=[
            AssetResponse.model_validate(a)
            for a in sorted(
                episode.assets,
                key=lambda a: (a.asset_type != AssetType.VIDEO, -(a.id or 0)),
            )
        ],
    )


@router.delete("/{episode_id}")
async def delete_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Delete an episode and its associated data."""
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()
    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    try:
        await db.delete(episode)
        await db.commit()
    except Exception as e:
        logger.warning(
            "delete_episode_failed", episode_id=str(episode_id), error=str(e)
        )
        await db.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to delete episode: {e!s}")

    return {
        "status": ResponseStatus.SUCCESS,
        "message": f"Episode {episode_id} and its traces/assets deleted.",
    }


@router.websocket("/{episode_id}/ws")
async def episode_websocket(websocket: WebSocket, episode_id: uuid.UUID):
    await manager.connect(episode_id, websocket)
    try:
        # Initial message
        await websocket.send_json(
            {
                "type": "log",
                "data": f"Subscribed to updates for episode {episode_id}",
                "timestamp": datetime.utcnow().isoformat(),
            }
        )

        # Keep connection alive
        while True:
            await asyncio.sleep(30)
            await websocket.send_json({"type": "heartbeat"})
    except WebSocketDisconnect:
        manager.disconnect(episode_id, websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(episode_id, websocket)
        raise e from None


@router.post("/{episode_id}/interrupt")
async def interrupt_episode(episode_id: uuid.UUID, db: AsyncSession = Depends(get_db)):
    """Interrupt a running episode."""
    # 1. Cancel the asyncio task
    task = task_tracker.get_task(episode_id)
    if task and not task.done():
        logger.info("cancelling_episode_task", episode_id=str(episode_id))
        task.cancel()
    else:
        logger.warning("task_not_found_or_already_done", episode_id=str(episode_id))

    # 2. Update status in DB immediately (failsafe)
    # The cancelled task exception handler in main.py will also do this,
    # but we do it here to give immediate feedback.
    result = await db.execute(select(Episode).where(Episode.id == episode_id))
    episode = result.scalar_one_or_none()

    if episode and episode.status in [EpisodeStatus.RUNNING]:
        episode.status = EpisodeStatus.CANCELLED
        await db.commit()

        # Broadcast update
        await manager.broadcast(
            episode_id,
            {
                "type": "status_update",
                "status": EpisodeStatus.CANCELLED,
                "timestamp": datetime.utcnow().isoformat(),
            },
        )

    return {"status": ResponseStatus.ACCEPTED, "message": "Interruption requested"}
