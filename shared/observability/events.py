import json
import time
from pathlib import Path
from typing import Any

from .schemas import BaseEvent


def emit_event(
    event: BaseEvent | dict[str, Any],
    agent_id: str | None = None,
) -> None:
    """
    Emits a structured event to a local events.jsonl file.
    This is used for observability signals during agent execution.

    Args:
        event: A BaseEvent model or a dictionary containing event-specific data.
        agent_id: Optional ID of the agent emitting the event (overrides model agent_id).
    """
    if isinstance(event, BaseEvent):
        # Update agent_id if explicitly provided
        if agent_id:
            event.agent_id = agent_id

        # Serialize model
        event_dict = event.model_dump(mode="json")
        # Add unix_timestamp if not present (BaseEvent doesn't have it explicitly to keep it clean)
        if "unix_timestamp" not in event_dict:
            event_dict["unix_timestamp"] = time.time()
    else:
        # Backward compatibility for raw dicts
        event_dict = event.copy()
        if "timestamp" not in event_dict:
            from datetime import UTC, datetime

            event_dict["timestamp"] = datetime.now(UTC).isoformat()
        if "unix_timestamp" not in event_dict:
            event_dict["unix_timestamp"] = time.time()
        if agent_id:
            event_dict["agent_id"] = agent_id

    try:
        events_file = Path("events.jsonl")
        with events_file.open("a") as f:
            f.write(json.dumps(event_dict) + "\n")
    except Exception as e:
        import sys

        print(f"Error emitting event: {e}", file=sys.stderr)
