import pytest

from shared.simulation.schemas import SimulationFailureMode
from worker.simulation.loop import SimulationLoop

# Mock XML for testing
TEST_XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 1"/>
    <geom name="floor" type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body name="target_box" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="box" size=".05 .05 .05" rgba="1 0 0 1" mass="1"/>
    </body>
    <body name="actuated_body" pos="0 0 1">
      <joint name="test_joint" type="hinge" axis="0 0 1"/>
      <geom type="box" size=".05 .05 .05" mass="1"/>
    </body>
    <body name="zone_goal_body" pos="0.5 0 0">
         <site name="zone_goal" type="box" size="0.1 0.1 0.1" rgba="0 1 0 0.3" group="1"/>
    </body>
    <body name="zone_forbid_body" pos="-0.5 0 0">
         <site name="zone_forbid_1" type="box" size="0.1 0.1 0.1" rgba="1 0 0 0.3" group="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="test_actuator" joint="test_joint" gear="1"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def sim_loop(tmp_path):
    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)
    loop = SimulationLoop(str(xml_path))
    return loop


def test_initialization(sim_loop):
    import mujoco

    assert sim_loop.backend.model is not None
    assert sim_loop.backend.data is not None
    # Check if name of forbidden zone is in the list
    assert "zone_forbid_1" in sim_loop.forbidden_sites


def test_step_simulation(sim_loop):
    # Initial Z height
    z_start = sim_loop.backend.data.qpos[2]  # free joint: x y z w x y z

    metrics = sim_loop.step({}, duration=0.1)

    # Check time advanced
    assert metrics.total_time > 0
    # Check gravity worked (box fell)
    assert sim_loop.backend.data.qpos[2] < z_start
    assert metrics.success is False  # Not reached goal yet


def test_metrics_collection(sim_loop):
    # Apply control to actuator
    # Gear is 1, so control 1.0 should apply 1.0 torque
    metrics = sim_loop.step({"test_actuator": 1.0}, duration=0.5)

    assert metrics.total_energy > 0
    assert metrics.max_velocity >= 0  # Target box might not move much but hinges will

    # Check velocity of target box specifically
    sim_loop.backend.data.qvel[0] = 5.0  # Set x-velocity of target box
    metrics = sim_loop.step({}, duration=0.01)
    assert metrics.max_velocity >= 5.0


def test_goal_zone_trigger(sim_loop):
    # move target to goal
    # Goal is at 0.5 0 0
    # Target is free joint. qpos[0:3] is pos.
    sim_loop.backend.data.qpos[0] = 0.5
    sim_loop.backend.data.qpos[1] = 0.0
    sim_loop.backend.data.qpos[2] = 0.0

    # Run step
    metrics = sim_loop.step({}, duration=0.01)
    assert metrics.success is True


def test_forbidden_zone_trigger(sim_loop):
    # Move target to forbidden zone (-0.5 0 0)
    sim_loop.backend.data.qpos[0] = -0.5
    sim_loop.backend.data.qpos[1] = 0.0
    sim_loop.backend.data.qpos[2] = 0.0

    # Run step
    metrics = sim_loop.step({}, duration=0.01)

    # Collision detection relies on contacts.
    # Just setting position might not generate contacts immediately if no step is running?
    # mj_step computes contacts.
    assert metrics.fail_reason == "collision_with_forbidden_zone"


def test_validation_hook_failure(tmp_path):
    from unittest.mock import patch

    from build123d import Box

    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)

    # Mock validate_and_price to return invalid
    with patch("worker.utils.dfm.validate_and_price") as mock_val:
        from unittest.mock import MagicMock

        mock_val.return_value = MagicMock(
            is_manufacturable=False, violations=["Part too large"]
        )

        # Pass a dummy component to trigger validation
        loop = SimulationLoop(str(xml_path), component=Box(1, 1, 1))

        metrics = loop.step({})
        assert metrics.success is False
        assert "validation_failed" in metrics.fail_reason
        assert "Part too large" in metrics.fail_reason
        # Check that it didn't step (total_time should be 0.0)
        assert metrics.total_time == 0.0


def test_timeout_configurable(tmp_path):
    """Test that configurable timeout works."""
    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)

    # Set a very short timeout (0.05s = 50ms)
    loop = SimulationLoop(str(xml_path), max_simulation_time=0.05)

    # Run for longer than timeout
    metrics = loop.step({}, duration=1.0)

    # Should timeout before reaching goal
    assert metrics.success is False
    assert metrics.fail_reason == "timeout_exceeded"
    assert metrics.total_time >= 0.05  # At least ran until timeout


def test_timeout_capped_at_30s(tmp_path):
    """Test that timeout cannot exceed 30 seconds (hard cap)."""
    from worker.simulation.loop import MAX_SIMULATION_TIME_SECONDS

    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)

    # Try to set a timeout longer than 30s
    loop = SimulationLoop(str(xml_path), max_simulation_time=60.0)

    # Should be capped at 30s
    assert loop.max_simulation_time == MAX_SIMULATION_TIME_SECONDS
    assert loop.max_simulation_time == 30.0


def test_target_fell_off_world(sim_loop):
    """Test failure detection when target falls below Z threshold."""
    # Set target Z to -3.0 (below -2.0 threshold)
    sim_loop.backend.data.qpos[2] = -3.0

    # Run step
    metrics = sim_loop.step({}, duration=0.01)

    assert metrics.success is False
    assert metrics.fail_reason == "target_fell_off_world"


def test_instability_detection(sim_loop):
    """Test failure detection when NaNs appear in simulation state."""
    import numpy as np

    # Inject NaN into qpos
    sim_loop.backend.data.qpos[0] = np.nan

    # Run step
    metrics = sim_loop.step({}, duration=0.01)

    assert metrics.success is False
    assert metrics.fail_reason == SimulationFailureMode.PHYSICS_INSTABILITY
