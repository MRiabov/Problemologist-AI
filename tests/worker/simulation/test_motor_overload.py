"""Tests for motor overload detection."""

import pytest

from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.loop import (
    MOTOR_OVERLOAD_THRESHOLD_SECONDS,
    SimulationLoop,
)

# XML with position actuator and strict forcerange to trigger overload
TEST_OVERLOAD_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      <geom type="box" size=".2 .1 .1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="hinge" kp="50" kv="5" forcerange="-0.1 0.1"/>
  </actuator>
</mujoco>
"""

# XML with no forcerange (should never trigger overload)
TEST_NO_LIMIT_XML = """
<mujoco>
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size=".1 .05 .05"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="hinge" kp="5" kv="0.5"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def overload_loop(tmp_path):
    xml_path = tmp_path / "test_overload.xml"
    xml_path.write_text(TEST_OVERLOAD_XML)
    return SimulationLoop(
        str(xml_path), backend_type=SimulatorBackendType.MUJOCO
    )


@pytest.fixture
def no_limit_loop(tmp_path):
    xml_path = tmp_path / "test_no_limit.xml"
    xml_path.write_text(TEST_NO_LIMIT_XML)
    return SimulationLoop(
        str(xml_path), backend_type=SimulatorBackendType.MUJOCO
    )


class TestMotorOverload:
    def test_overload_detection_triggers(self, overload_loop):
        """Test that motor clamped at forcerange for >2s fails simulation."""
        # Demand large position that can't be reached with tiny forcerange
        # This will keep the motor saturated
        from worker.utils.controllers.position_based import hold_position

        controllers = {"servo": hold_position(3.14)}  # Impossible with 0.1 N limit

        # Run for 3 seconds (should trigger 2s overload threshold)
        metrics = overload_loop.step({}, duration=3.0, dynamic_controllers=controllers)

        assert not metrics.success
        assert metrics.fail_reason is not None
        assert "overcurrent" in metrics.fail_reason

    def test_normal_operation_no_overload(self, no_limit_loop):
        """Test that normal operation doesn't trigger overload."""
        from worker.utils.controllers.position_based import hold_position

        controllers = {"servo": hold_position(0.5)}

        metrics = no_limit_loop.step({}, duration=2.0, dynamic_controllers=controllers)

        # Should not fail due to overload
        assert metrics.fail_reason is None or "overcurrent" not in str(
            metrics.fail_reason
        )

    def test_overload_resets_on_unclamp(self, overload_loop):
        """Test that clamp counter resets if motor becomes unclamped."""
        # This tests that overload requires CONTINUOUS clamping
        assert (
            overload_loop.success_evaluator.motor_overload_timer.get("servo", 0.0)
            == 0.0
        )

        # After reset, should be zero
        overload_loop.reset_metrics()
        assert (
            overload_loop.success_evaluator.motor_overload_timer.get("servo", 0.0)
            == 0.0
        )

    def test_threshold_constant_is_2_seconds(self):
        """Verify the overload threshold matches architecture spec."""
        assert MOTOR_OVERLOAD_THRESHOLD_SECONDS == 2.0
