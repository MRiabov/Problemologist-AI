import asyncio
import uuid
from collections.abc import Awaitable
from datetime import UTC, datetime

import structlog

from controller.api.manager import manager, task_tracker
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode, Trace
from shared.enums import EpisodeStatus, TraceType
from shared.models.schemas import EpisodeMetadata

logger = structlog.get_logger(__name__)


async def _mark_benchmark_episode_cancelled(
    session_id: uuid.UUID,
) -> dict | None:
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, session_id)
        if episode is None:
            logger.warning(
                "benchmark_cancelled_episode_missing",
                episode_id=str(session_id),
            )
            return None

        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        metadata.detailed_status = EpisodeStatus.CANCELLED.value
        episode.status = EpisodeStatus.CANCELLED
        episode.metadata_vars = metadata.model_dump()
        db.add(
            Trace(
                episode_id=session_id,
                trace_type=TraceType.LOG,
                content="Benchmark session cancelled by user",
            )
        )
        await db.commit()
        return episode.metadata_vars


async def _run_tracked_benchmark_task(
    session_id: uuid.UUID,
    task_coro: Awaitable[object],
) -> None:
    try:
        await task_coro
    except asyncio.CancelledError:
        logger.info("benchmark_run_cancelled", episode_id=str(session_id))
        metadata_vars = await _mark_benchmark_episode_cancelled(session_id)
        await manager.broadcast(
            session_id,
            {
                "type": "status_update",
                "status": EpisodeStatus.CANCELLED,
                "metadata_vars": metadata_vars,
                "timestamp": datetime.now(UTC).isoformat(),
            },
        )
        raise
    finally:
        task_tracker.remove_task(session_id)


def launch_tracked_benchmark_task(
    session_id: uuid.UUID,
    task_coro: Awaitable[object],
) -> asyncio.Task:
    task = asyncio.create_task(_run_tracked_benchmark_task(session_id, task_coro))
    task_tracker.register_task(session_id, task)
    return task
