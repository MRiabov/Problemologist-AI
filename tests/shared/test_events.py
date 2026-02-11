import json
from pathlib import Path

from shared.observability.events import emit_event
from shared.observability.schemas import LogicFailureEvent


def test_emit_event():
    event_file = Path("events.jsonl")
    if event_file.exists():
        event_file.unlink()

    try:
        event = LogicFailureEvent(
            file_path="test.py",
            constraint_name="test_constraint",
            error_message="test error",
            agent_id="test_agent",
        )
        emit_event(event)

        assert event_file.exists()
        with event_file.open("r") as f:
            lines = f.readlines()
            assert len(lines) == 1
            event_data = json.loads(lines[0])
            assert event_data["event_type"] == "logic_failure"
            assert event_data["file_path"] == "test.py"
            assert event_data["agent_id"] == "test_agent"
            assert "timestamp" in event_data
            assert "unix_timestamp" in event_data
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
