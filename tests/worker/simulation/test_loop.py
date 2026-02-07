import pytest
import mujoco
import numpy as np
from worker.simulation.loop import SimulationLoop, SimulationMetrics

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
    <body name="zone_goal_body" pos="0.5 0 0">
         <geom name="zone_goal" type="box" size="0.1 0.1 0.1" rgba="0 1 0 0.3" group="1"/>
    </body>
    <body name="zone_forbid_body" pos="-0.5 0 0">
         <geom name="zone_forbid_1" type="box" size="0.1 0.1 0.1" rgba="1 0 0 0.3" group="1"/>
    </body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def sim_loop(tmp_path):
    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)
    loop = SimulationLoop(str(xml_path))
    return loop


def test_initialization(sim_loop):
    assert sim_loop.model is not None
    assert sim_loop.data is not None
    assert "zone_forbid_1" in sim_loop.forbidden_geoms


def test_step_simulation(sim_loop):
    # Initial Z height
    z_start = sim_loop.data.qpos[2]  # free joint: x y z w x y z

    metrics = sim_loop.step({}, duration=0.1)

    # Check time advanced
    assert metrics.total_time > 0
    # Check gravity worked (box fell)
    # Note: reset is needed if step modifies state. step() does NOT reset state, only metrics.
    # Current loop implementation does not reset state.
    assert sim_loop.data.qpos[2] < z_start
    assert metrics.success is False  # Not reached goal yet


def test_goal_zone_trigger(sim_loop):
    # move target to goal
    # Goal is at 0.5 0 0
    # Target is free joint. qpos[0:3] is pos.
    sim_loop.data.qpos[0] = 0.5
    sim_loop.data.qpos[1] = 0.0
    sim_loop.data.qpos[2] = 0.0

    # Run step
    metrics = sim_loop.step({}, duration=0.01)
    assert metrics.success is True


def test_forbidden_zone_trigger(sim_loop):
    # Move target to forbidden zone (-0.5 0 0)
    sim_loop.data.qpos[0] = -0.5
    sim_loop.data.qpos[1] = 0.0
    sim_loop.data.qpos[2] = 0.0

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
        mock_val.return_value = MagicMock(is_manufacturable=False, violations=["Part too large"])

        # Pass a dummy component to trigger validation
        loop = SimulationLoop(str(xml_path), component=Box(1, 1, 1))

        metrics = loop.step({})
        assert metrics.success is False
        assert "validation_failed" in metrics.fail_reason
        assert "Part too large" in metrics.fail_reason
        # Check that it didn't step (total_time should be 0.0)
        assert metrics.total_time == 0.0
