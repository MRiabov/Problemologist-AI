import json
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


def emit_event(
    event_type: str, data: dict[str, Any], agent_id: str | None = None
) -> None:
    """
    Emits a structured event to a local events.jsonl file.
     This is used for observability signals during agent execution.

    Args:
        event_type: The type of event (e.g. 'component_usage', 'metric').
        data: A dictionary containing event-specific data.
        agent_id: Optional ID of the agent emitting the event.
    """
    event = {
        "event_type": event_type,
        "data": data,
        "timestamp": datetime.now(UTC).isoformat(),
        "unix_timestamp": time.time(),
    }

    if agent_id:
        event["agent_id"] = agent_id

    # The worker runtime executor runs scripts in the workspace root.
    # We write to events.jsonl in the current working directory.
    # Note: We append to the file.
    try:
        events_file = Path("events.jsonl")
        with events_file.open("a") as f:
            f.write(json.dumps(event) + "\n")
    except Exception as e:
        # We don't want observability to crash the main execution,
        # but we should probably log it if we have a logger available.
        # For now, just a quiet fail or print to stderr.
        import sys

        print(f"Error emitting event: {e}", file=sys.stderr)
