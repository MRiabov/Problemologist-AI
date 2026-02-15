import pytest
import numpy as np
from unittest.mock import MagicMock, patch
from shared.simulation.backends import SimulatorBackendType, StressField, BodyState
from worker.simulation.loop import SimulationLoop


@pytest.mark.integration
def test_fem_breakage_failure(tmp_path):
    """
    Verify that exceeding ultimate stress triggers PART_BREAKAGE failure in Genesis.
    """
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco model='test'><worldbody/></mujoco>")

    # Mock the backend to avoid requiring a real GPU/Genesis install for this logic check
    with patch("worker.simulation.loop.get_physics_backend") as mock_get:
        backend = MagicMock()
        mock_get.return_value = backend

        # Default mock setup
        backend.get_all_body_names.return_value = ["test_part"]
        backend.get_all_site_names.return_value = []
        backend.get_all_actuator_names.return_value = []
        backend.get_body_state.return_value = BodyState(
            pos=(0, 0, 0), quat=(1, 0, 0, 0), vel=(0, 0, 0), angvel=(0, 0, 0)
        )

        # Mock time progression
        def mock_step(dt):
            mock_step.time += dt
            return MagicMock(time=mock_step.time, success=True)

        mock_step.time = 0.0
        backend.step.side_effect = mock_step

        # Mock stress: 400 MPa exceeds Aluminum ultimate stress (310 MPa)
        backend.get_stress_field.return_value = StressField(
            nodes=np.zeros((1, 3)), stress=np.array([400e6])
        )

        loop = SimulationLoop(str(xml_path), backend_type=SimulatorBackendType.GENESIS)

        # Run simulation
        metrics = loop.step(control_inputs={}, duration=0.1)

        # Verify failure
        assert metrics.success is False
        assert metrics.fail_reason is not None
        assert "PART_BREAKAGE:test_part" in metrics.fail_reason
        assert len(metrics.stress_summaries) > 0
        assert metrics.stress_summaries[0].max_von_mises_pa == 400e6


if __name__ == "__main__":
    pytest.main([__file__])
