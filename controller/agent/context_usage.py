from datetime import UTC, datetime
from uuid import UUID

import structlog

from controller.api.manager import manager
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.models.schemas import EpisodeMetadata

logger = structlog.get_logger(__name__)


async def update_episode_context_usage(
    episode_id: str,
    used_chars: int,
    max_chars: int,
) -> None:
    """Persist current context usage in episode metadata and broadcast status update."""
    if not episode_id:
        return
    try:
        episode_uuid = UUID(str(episode_id))

        session_factory = get_sessionmaker()
        async with session_factory() as db:
            episode = await db.get(Episode, episode_uuid)
            if not episode:
                return

            metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
            additional_info = dict(metadata.additional_info or {})
            current = additional_info.get("context_usage")
            next_payload = {
                "used_chars": int(used_chars),
                "max_chars": int(max_chars),
                "utilization_ratio": (float(used_chars) / float(max_chars))
                if max_chars > 0
                else 0.0,
            }

            if current == next_payload:
                return

            additional_info["context_usage"] = next_payload
            metadata.additional_info = additional_info
            episode.metadata_vars = metadata.model_dump()
            await db.commit()

            await manager.broadcast(
                episode_uuid,
                {
                    "type": "status_update",
                    "status": episode.status,
                    "metadata_vars": episode.metadata_vars,
                    "timestamp": datetime.now(UTC).isoformat(),
                },
            )
    except Exception as exc:
        logger.warning("context_usage_metadata_update_failed", error=str(exc))
