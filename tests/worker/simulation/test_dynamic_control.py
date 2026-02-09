import pytest
import mujoco
import numpy as np
from worker.simulation.loop import SimulationLoop
from worker.utils.controllers.time_based import constant

# Mock XML with an actuator
TEST_XML = """
<mujoco>
  <worldbody>
    <body name="actuated_body" pos="0 0 1">
      <joint name="test_joint" type="hinge" axis="0 0 1"/>
      <geom type="box" size=".05 .05 .05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="test_actuator" joint="test_joint" gear="1"/>
  </actuator>
</mujoco>
"""


@pytest.fixture
def sim_loop(tmp_path):
    xml_path = tmp_path / "test_dynamic.xml"
    xml_path.write_text(TEST_XML)
    return SimulationLoop(str(xml_path))


def test_dynamic_controllers(sim_loop):
    # Constant 10.0 controller
    controllers = {"test_actuator": constant(10.0)}

    # Run for a very short time
    sim_loop.step({}, duration=0.01, dynamic_controllers=controllers)

    # Check that ctrl was set to 10.0
    assert sim_loop.data.ctrl[0] == 10.0


def test_dynamic_controllers_time_varying(sim_loop):
    # Controller that returns time * 10
    def time_controller(t):
        return t * 10.0

    controllers = {"test_actuator": time_controller}

    # Step simulation multiple times
    sim_loop.step({}, duration=0.1, dynamic_controllers=controllers)

    # At t=0.1, ctrl should be roughly 1.0 (if applied at the end/during)
    # The last applied control in the loop was at t_prev
    assert sim_loop.data.ctrl[0] > 0
    assert sim_loop.data.ctrl[0] <= 1.0
