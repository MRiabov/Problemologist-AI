import json
from pathlib import Path

from shared.cots.parts.motors import ServoMotor


def test_servo_motor_emission():
    event_file = Path("events.jsonl")
    if event_file.exists():
        event_file.unlink()

    try:
        # Instantiate ServoMotor
        servo = ServoMotor(size="SG90")

        # Verify metadata
        assert servo.metadata["part_number"] == "SG90"
        assert servo.label == "motor_SG90"

        # Verify event emission
        assert event_file.exists()
        with event_file.open("r") as f:
            lines = f.readlines()
            assert len(lines) == 1
            event = json.loads(lines[0])
            assert event["event_type"] == "component_usage"
            assert event["category"] == "motor"
            assert event["part_number"] == "SG90"
            assert event["price"] == 2.50
    finally:
        if event_file.exists():
            event_file.unlink()


if __name__ == "__main__":
    try:
        test_servo_motor_emission()
        print("Refactor verification test passed!")
    except Exception as e:
        print(f"Refactor verification test failed: {e}")
        import traceback

        traceback.print_exc()
        exit(1)
