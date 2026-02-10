import json
from pathlib import Path
from typing import Any

EVENTS_FILE = Path("events.jsonl")


def clear_emitted_events():
    """Truncates the events.jsonl file to ensure a clean state for tests."""
    if EVENTS_FILE.exists():
        EVENTS_FILE.unlink()


def get_emitted_events() -> list[dict[str, Any]]:
    """Reads events.jsonl and returns a list of parsed events."""
    if not EVENTS_FILE.exists():
        return []

    events = []
    with EVENTS_FILE.open("r") as f:
        for line in f:
            if line.strip():
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return events


def assert_event_emitted(event_type: str, **filters):
    """
    Asserts that at least one event of the given type was emitted,
    optionally matching additional filters in the event data.
    """
    events = get_emitted_events()
    matching_events = [e for e in events if e.get("event_type") == event_type]

    if not matching_events:
        raise AssertionError(
            f"No event of type '{event_type}' was emitted. Found: {[e.get('event_type') for e in events]}"
        )

    if filters:
        for event in matching_events:
            match = True
            for key, value in filters.items():
                if event.get(key) != value:
                    match = False
                    break
            if match:
                return  # Found a match

        raise AssertionError(
            f"Event of type '{event_type}' found, but none matched filters: {filters}"
        )
