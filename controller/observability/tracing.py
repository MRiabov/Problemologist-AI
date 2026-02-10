import uuid
from typing import Any

from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace
from shared.enums import TraceType
from shared.observability.schemas import BaseEvent


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
