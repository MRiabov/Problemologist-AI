import uuid
from typing import Any

from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace
from shared.enums import TraceType


async def record_worker_events(
    episode_id: str | uuid.UUID,
    events: list[dict[str, Any]],
    langfuse_trace_id: str | None = None,
) -> None:
    """
    Persists structured events from worker execution into the database.

    Args:
        episode_id: The ID of the current episode (can be string or UUID).
        events: List of event dictionaries from worker response.
        langfuse_trace_id: Optional Langfuse trace ID to link the event.
    """
    if not events:
        return

    # Convert episode_id to UUID if it's a string
    if isinstance(episode_id, str):
        try:
            episode_uuid = uuid.UUID(episode_id)
        except ValueError:
            # If not a UUID, we might have trouble with foreign key constraints
            # unless the DB allows arbitrary strings. In our schema it's UUID.
            return
    else:
        episode_uuid = episode_id

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        for event_data in events:
            trace = Trace(
                episode_id=episode_uuid,
                trace_type=TraceType.EVENT,
                name=event_data.get("event_type", "generic_event"),
                content=str(event_data.get("data", {})),
                metadata_vars=event_data,
                langfuse_trace_id=langfuse_trace_id,
            )
            db.add(trace)

        await db.commit()
