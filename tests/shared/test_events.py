import json
from pathlib import Path

from shared.observability.events import emit_event


def test_emit_event():
    event_file = Path("events.jsonl")
    if event_file.exists():
        event_file.unlink()

    try:
        emit_event("test_event", {"foo": "bar"}, agent_id="test_agent")

        assert event_file.exists()
        with event_file.open("r") as f:
            lines = f.readlines()
            assert len(lines) == 1
            event = json.loads(lines[0])
            assert event["event_type"] == "test_event"
            assert event["data"] == {"foo": "bar"}
            assert event["agent_id"] == "test_agent"
            assert "timestamp" in event
            assert "unix_timestamp" in event
    finally:
        if event_file.exists():
            event_file.unlink()


if __name__ == "__main__":
    try:
        test_emit_event()
        print("Unit test passed!")
    except Exception as e:
        print(f"Unit test failed: {e}")
        exit(1)
