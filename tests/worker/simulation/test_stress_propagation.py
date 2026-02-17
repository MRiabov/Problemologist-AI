import pytest
from unittest.mock import MagicMock
from worker.simulation.loop import SimulationLoop
from shared.simulation.backends import StepResult, PhysicsBackend, BodyState

TEST_XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 1"/>
    <geom name="floor" type="plane" size="1 1 0.1" rgba=".9 .9 .9 1"/>
    <body name="target_box" pos="0 0 0.5">
      <joint type="free"/>
      <geom type="box" size=".05 .05 .05" rgba="1 0 0 1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
"""

def test_stress_propagation(tmp_path):
    # Setup
    xml_path = tmp_path / "test.xml"
    xml_path.write_text(TEST_XML)

    loop = SimulationLoop(str(xml_path))

    # Mock backend
    mock_backend = MagicMock(spec=PhysicsBackend)
    loop.backend = mock_backend

    # Configure mock step to return specific max_stress
    mock_backend.step.return_value = StepResult(
        time=0.1,
        success=True,
        max_stress=123.45
    )

    # Mock other backend methods called during step
    mock_backend.get_all_actuator_names.return_value = []
    mock_backend.get_all_body_names.return_value = ["target_box"]

    # Return valid BodyState
    mock_backend.get_body_state.return_value = BodyState(
        pos=(0, 0, 0),
        quat=(1, 0, 0, 0),
        vel=(0, 0, 0),
        angvel=(0, 0, 0)
    )

    mock_backend.check_collision.return_value = False

    # Mock metric_collector to verify update call
    loop.metric_collector = MagicMock()

    # Run step
    loop.step({}, duration=0.01)

    # Verify
    # metric_collector.update(dt, energy, target_vel, max_stress)
    loop.metric_collector.update.assert_called()
    args, _ = loop.metric_collector.update.call_args

    # Check that max_stress (4th argument) is passed correctly
    assert args[3] == 123.45, f"Expected max_stress 123.45, got {args[3]}"
