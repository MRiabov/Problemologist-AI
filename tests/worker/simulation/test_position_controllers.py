"""Tests for position-based motor controllers."""

import mujoco
import pytest

from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.loop import SimulationLoop
from worker.utils.controllers.position_based import hold_position, oscillate, waypoint

# XML with a POSITION actuator (not motor)
# CRITICAL: PD gains must be tuned relative to inertia to avoid numerical instability.
# - Higher inertia = can use higher kp
# - Low mass/inertia with high kp causes simulation explosion
TEST_POSITION_XML = """
<mujoco>
  <option gravity="0 0 0" timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.1"/>
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
def sim_loop_position(tmp_path):
    xml_path = tmp_path / "test_position.xml"
    xml_path.write_text(TEST_POSITION_XML)
    return SimulationLoop(str(xml_path), backend_type=SimulatorBackendType.MUJOCO)


class TestWaypointController:
    def test_waypoint_empty_schedule(self):
        ctrl = waypoint([])
        assert ctrl(0) == 0.0
        assert ctrl(10) == 0.0

    def test_waypoint_single_point(self):
        ctrl = waypoint([(0, 0.5)])
        assert ctrl(0) == 0.5
        assert ctrl(100) == 0.5

    def test_waypoint_multiple_points(self):
        # At t=0: 0 rad, t=2: π/4, t=5: 0
        schedule = [(0, 0), (2, 0.785), (5, 0)]
        ctrl = waypoint(schedule)

        assert ctrl(0) == 0  # At t=0
        assert ctrl(1) == 0  # Before t=2, still at 0
        assert ctrl(2) == 0.785  # At t=2
        assert ctrl(3) == 0.785  # Between t=2 and t=5
        assert ctrl(5) == 0  # At t=5
        assert ctrl(10) == 0  # After last waypoint

    def test_waypoint_unsorted_input(self):
        # Even if input is unsorted, should work
        schedule = [(5, 0), (0, 0), (2, 0.785)]
        ctrl = waypoint(schedule)

        assert ctrl(3) == 0.785  # Should still find correct waypoint


class TestHoldPosition:
    def test_hold_position_constant(self):
        ctrl = hold_position(1.0)
        assert ctrl(0) == 1.0
        assert ctrl(100) == 1.0
        assert ctrl(-50) == 1.0


class TestOscillate:
    def test_oscillate_at_zero(self):
        ctrl = oscillate(center=0, amplitude=1.0, frequency=1.0, phase=0)
        # sin(0) = 0
        assert abs(ctrl(0) - 0) < 1e-6

    def test_oscillate_at_quarter_period(self):
        ctrl = oscillate(center=0, amplitude=1.0, frequency=1.0, phase=0)
        # sin(π/2) = 1 at t=0.25 for freq=1Hz
        assert abs(ctrl(0.25) - 1.0) < 1e-6

    def test_oscillate_with_center(self):
        ctrl = oscillate(center=2.0, amplitude=0.5, frequency=1.0, phase=0)
        # At t=0.25: 2.0 + 0.5 * sin(π/2) = 2.5
        assert abs(ctrl(0.25) - 2.5) < 1e-6


class TestPositionActuatorIntegration:
    def test_hold_position_moves_joint(self, sim_loop_position):
        """Test that position actuator actually moves the joint."""
        target = 0.5  # radians
        controllers = {"servo": hold_position(target)}

        # Run for a bit
        sim_loop_position.step({}, duration=1.0, dynamic_controllers=controllers)

        # The joint should have moved towards the target
        joint_id = mujoco.mj_name2id(
            sim_loop_position.backend.model, mujoco.mjtObj.mjOBJ_JOINT, "hinge"
        )
        # qpos for hinge is 1D
        qpos_addr = sim_loop_position.backend.model.jnt_qposadr[joint_id]
        actual_pos = sim_loop_position.backend.data.qpos[qpos_addr]

        # Should be close to target (PD control with kp=100, kv=10)
        assert abs(actual_pos - target) < 0.1, f"Expected ~{target}, got {actual_pos}"

    def test_waypoint_sequence(self, sim_loop_position):
        """Test waypoint controller moves through schedule."""
        schedule = [(0, 0.3), (0.5, 0.6)]
        controllers = {"servo": waypoint(schedule)}

        # Run for 1 second
        sim_loop_position.step({}, duration=1.0, dynamic_controllers=controllers)

        # At end, should be near 0.6
        joint_id = mujoco.mj_name2id(
            sim_loop_position.backend.model, mujoco.mjtObj.mjOBJ_JOINT, "hinge"
        )
        qpos_addr = sim_loop_position.backend.model.jnt_qposadr[joint_id]
        actual_pos = sim_loop_position.backend.data.qpos[qpos_addr]

        assert abs(actual_pos - 0.6) < 0.15, f"Expected ~0.6, got {actual_pos}"
