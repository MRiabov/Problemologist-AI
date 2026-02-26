import uuid
from pathlib import Path
from typing import Any

import structlog
from sqlalchemy import select

from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Trace
from shared.enums import AssetType, TraceType
from shared.observability.schemas import BaseEvent

logger = structlog.get_logger(__name__)


async def record_worker_events(
    episode_id: str | uuid.UUID,
    events: list[dict[str, Any] | BaseEvent],
    langfuse_trace_id: str | None = None,
) -> None:
    """
    Persists structured events from worker execution into the database.

    Args:
        episode_id: The ID of the current episode (can be string or UUID).
        events: List of event dictionaries or BaseEvent models.
        langfuse_trace_id: Optional Langfuse trace ID to link the event.
    """
    if not events:
        return

    # Convert episode_id to UUID if it's a string
    if isinstance(episode_id, str):
        try:
            episode_uuid = uuid.UUID(episode_id)
        except ValueError:
            return
    else:
        episode_uuid = episode_id

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        for event in events:
            if isinstance(event, BaseEvent):
                # Serialize model
                event_dict = event.model_dump(mode="json")
                name = event_dict.get("event_type", "generic_event")
                # content can be a string representation for searchability
                content = str(event_dict.get("data", event_dict))
                metadata = event_dict
            else:
                name = event.get("event_type", "generic_event")
                content = str(event.get("data", {}))
                metadata = event

            trace = Trace(
                episode_id=episode_uuid,
                trace_type=TraceType.EVENT,
                name=name,
                content=content,
                metadata_vars=metadata,
                langfuse_trace_id=langfuse_trace_id,
            )
            db.add(trace)

        await db.commit()


async def sync_asset(
    episode_id: str | uuid.UUID,
    path: str | Path,
    content: str | None = None,
    asset_type: AssetType | None = None,
) -> Asset | None:
    """
    Syncs or creates an Asset record for a file change.

    Args:
        episode_id: The ID of the current episode.
        path: Path to the asset (relative to worker root).
        content: Optional content snippet or full content to cache.
        asset_type: Optional explicit AssetType. If None, inferred from extension.
    """
    if isinstance(episode_id, str):
        try:
            # Strip common prefixes like INT-XXX- or sim-
            clean_id = episode_id
            if "-" in episode_id and not episode_id.startswith("{"):
                parts = episode_id.split("-")
                # If first part is not numeric and not a uuid part, it might be a prefix
                if len(parts) > 1 and len(parts[0]) <= 4:
                    # Check if the rest looks like a UUID or at least has enough parts
                    potential_uuid = "-".join(parts[1:])
                    if len(potential_uuid) >= 32:
                        clean_id = potential_uuid

            episode_uuid = uuid.UUID(clean_id)
        except ValueError:
            logger.warning(
                "sync_asset_uuid_invalid", episode_id=episode_id, path=str(path)
            )
            return None
    else:
        episode_uuid = episode_id

    if asset_type is None:
        p = Path(path)
        ext = p.suffix.lower()
        if ext == ".py":
            asset_type = AssetType.PYTHON
        elif ext in (".xml", ".mjcf"):
            asset_type = AssetType.MJCF
        elif ext in (".png", ".jpg", ".jpeg", ".webp"):
            asset_type = AssetType.IMAGE
        elif ext in (".glb", ".gltf"):
            asset_type = AssetType.GLB
        elif ext in (".stl",):
            asset_type = AssetType.STL
        elif ext in (".step", ".stp"):
            asset_type = AssetType.STEP
        elif ext in (".mp4", ".mov", ".avi", ".webm"):
            asset_type = AssetType.VIDEO
        elif ext in (".md", ".txt"):
            asset_type = AssetType.MARKDOWN
        elif ext in (".json", ".yaml", ".yml"):
            asset_type = AssetType.OTHER
        else:
            asset_type = AssetType.OTHER

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        # Check if asset already exists
        stmt = select(Asset).where(
            Asset.episode_id == episode_uuid, Asset.s3_path == path
        )
        res = await db.execute(stmt)
        asset = res.scalar_one_or_none()

        if asset:
            asset.content = content
            asset.asset_type = asset_type
            logger.info(
                "sync_asset_updated",
                episode_id=str(episode_uuid),
                path=str(path),
                type=asset_type,
            )
        else:
            asset = Asset(
                episode_id=episode_uuid,
                s3_path=str(path),
                content=content,
                asset_type=asset_type,
            )
            db.add(asset)
            logger.info(
                "sync_asset_created",
                episode_id=str(episode_uuid),
                path=str(path),
                type=asset_type,
            )

        await db.commit()
        await db.refresh(asset)
        return asset
