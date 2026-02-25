import xml.etree.ElementTree as ET

from shared.cots.parts.motors import ServoMotor
from worker_heavy.simulation.builder import SceneCompiler


def test_add_actuator_cots_derivation_sg90():
    """Test that add_actuator derives params from SG90 COTS data."""
    compiler = SceneCompiler()

    # Heuristic: find SG90 in name
    compiler.add_actuator(
        name="Servo_SG90_joint", joint="joint1", actuator_type="position"
    )

    xml_str = ET.tostring(compiler.root, encoding="utf-8").decode()
    root = ET.fromstring(xml_str)
    actuator = root.find(".//actuator/position")

    assert actuator is not None
    assert actuator.get("name") == "Servo_SG90_joint"

    # Check derived values
    kp = float(actuator.get("kp"))
    kv = float(actuator.get("kv"))
    forcerange = actuator.get("forcerange")

    expected_torque = ServoMotor.motor_data["SG90"]["torque_nm"]
    expected_kp = expected_torque / 0.2
    expected_kv = expected_kp * 0.1

    assert abs(kp - expected_kp) < 1e-4
    assert abs(kv - expected_kv) < 1e-4

    fr_vals = [float(x) for x in forcerange.split()]
    assert abs(fr_vals[0] - (-expected_torque)) < 1e-4
    assert abs(fr_vals[1] - expected_torque) < 1e-4


def test_add_actuator_cots_id_direct():
    """Test that passing cots_id directly works."""
    compiler = SceneCompiler()

    compiler.add_actuator(name="custom_name", joint="joint1", cots_id="MG996R")

    xml_str = ET.tostring(compiler.root, encoding="utf-8").decode()
    root = ET.fromstring(xml_str)
    actuator = root.find(".//actuator/position")

    assert actuator is not None
    assert actuator.get("name") == "custom_name"

    expected_torque = ServoMotor.motor_data["MG996R"]["torque_nm"]
    forcerange = actuator.get("forcerange")
    fr_vals = [float(x) for x in forcerange.split()]
    assert abs(fr_vals[1] - expected_torque) < 1e-4


def test_add_actuator_ds3218():
    """Test derivation for DS3218 motor."""
    compiler = SceneCompiler()

    compiler.add_actuator(name="motor_DS3218", joint="joint_ds")

    xml_str = ET.tostring(compiler.root, encoding="utf-8").decode()
    root = ET.fromstring(xml_str)
    actuator = root.find(".//actuator/position")

    assert actuator is not None
    expected_torque = ServoMotor.motor_data["DS3218"]["torque_nm"]
    fr_vals = [float(x) for x in actuator.get("forcerange").split()]
    assert abs(fr_vals[1] - expected_torque) < 1e-4


def test_add_actuator_manual_override():
    """Test that manual params override COTS derivation."""
    compiler = SceneCompiler()

    compiler.add_actuator(
        name="Servo_SG90_custom",
        joint="joint1",
        kp=50.0,
        kv=5.0,
        forcerange=(-1.0, 1.0),
    )

    xml_str = ET.tostring(compiler.root, encoding="utf-8").decode()
    root = ET.fromstring(xml_str)
    actuator = root.find(".//actuator/position")

    assert float(actuator.get("kp")) == 50.0
    assert float(actuator.get("kv")) == 5.0
    assert actuator.get("forcerange") == "-1.0 1.0"


def test_add_actuator_unknown_motor():
    """Test fallback for unknown motors."""
    compiler = SceneCompiler()

    compiler.add_actuator(name="Unknown_Motor", joint="joint1")

    xml_str = ET.tostring(compiler.root, encoding="utf-8").decode()
    root = ET.fromstring(xml_str)
    actuator = root.find(".//actuator/position")

    # Should use defaults from method signature
    assert float(actuator.get("kp")) == 10.0
    assert float(actuator.get("kv")) == 1.0
    assert actuator.get("forcerange") is None
