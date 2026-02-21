from unittest.mock import MagicMock, patch

from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.loop import SimulationLoop


class TestSimulationLoopStress:
    @patch("worker_heavy.simulation.loop.get_physics_backend")
    def test_stress_collection(self, mock_get_backend):
        # Setup mock backend
        mock_backend = MagicMock()
        # Prevent magicmock from returning a mock for timestep/model attributes
        # which would cause dt to be a mock and int(duration/dt) to be 1.
        del mock_backend.timestep
        del mock_backend.model
        mock_get_backend.return_value = mock_backend

        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["target_box"]
        mock_backend.get_body_state.return_value = MagicMock(
            pos=(0, 0, 0), vel=(0, 0, 0)
        )
        mock_backend.step.return_value = MagicMock(time=0.1, success=True)

        # Define sequential stress values to return
        mock_backend.get_max_stress.side_effect = [100.0, 500.0, 300.0]

        loop = SimulationLoop(
            xml_path="dummy.xml", backend_type=SimulatorBackendType.GENESIS
        )

        # Run 3 steps
        # SimulationLoop uses dt=0.002
        # duration=0.006 should result in 3 steps
        metrics = loop.step(control_inputs={}, duration=0.006)

        # Verify get_max_stress was called
        assert mock_backend.get_max_stress.call_count >= 3

        # MetricCollector should track the maximum
        assert metrics.max_stress == 500.0
