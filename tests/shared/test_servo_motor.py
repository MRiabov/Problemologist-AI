import pytest
from unittest.mock import patch
from shared.cots.parts.motors import ServoMotor
from shared.observability.schemas import ComponentUsageEvent


def test_servo_motor_emission():
    with patch("shared.cots.base.emit_event") as mock_emit:
        # Instantiate ServoMotor
        servo = ServoMotor(size="SG90")

        # Verify metadata
        assert servo.metadata.cots_id == "SG90"
        assert servo.label == "motor_SG90"

        # Verify event emission
        mock_emit.assert_called_once()
        event = mock_emit.call_args[0][0]
        assert isinstance(event, ComponentUsageEvent)
        assert event.category == "motor"
        assert event.part_number == "SG90"
        assert event.price == 2.50
        assert event.weight_g == 9.0


def test_servo_motor_mg996r():
    with patch("shared.cots.base.emit_event") as mock_emit:
        servo = ServoMotor(size="MG996R")
        assert servo.metadata.cots_id == "MG996R"
        assert servo.price == 12.00
        assert servo.weight_g == 55.0


def test_servo_motor_unknown():
    with patch("shared.cots.base.emit_event") as mock_emit:
        servo = ServoMotor(size="UNKNOWN_SIZE")
        assert servo.failed is True
