
import pytest
from unittest.mock import MagicMock, patch
from worker.simulation.loop import SimulationLoop
from shared.models.simulation import StressSummary
from shared.simulation.schemas import SimulatorBackendType

@pytest.fixture
def mock_backend_loop(tmp_path):
    # create a dummy file
    xml_path = tmp_path / "test.xml"
    xml_path.touch()

    # Init loop with mocked backend
    with patch("worker.simulation.loop.get_physics_backend") as mock_get_backend:
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend

        # Setup mock backend behavior
        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_all_body_names.return_value = ["target_box"]
        mock_backend.get_all_actuator_names.return_value = []

        # Mock step return
        step_res = MagicMock()
        step_res.time = 0.1
        step_res.success = True
        mock_backend.step.return_value = step_res

        # Mock stress summaries
        mock_backend.get_stress_summaries.return_value = [
            StressSummary(
                part_label="part1",
                max_von_mises_pa=100.0,
                mean_von_mises_pa=50.0,
                safety_factor=2.0,
                location_of_max=(0,0,0),
                utilization_pct=50.0
            ),
            StressSummary(
                part_label="part2",
                max_von_mises_pa=200.0,
                mean_von_mises_pa=100.0,
                safety_factor=1.5,
                location_of_max=(0,0,0),
                utilization_pct=75.0
            )
        ]

        # Mock other needed methods
        mock_backend.get_actuator_state.return_value = MagicMock(ctrl=0, velocity=0, force=0)
        mock_backend.get_body_state.return_value = MagicMock(pos=[0,0,0], vel=[0,0,0])
        mock_backend.get_particle_positions.return_value = None

        loop = SimulationLoop(str(xml_path), backend_type=SimulatorBackendType.GENESIS)
        return loop, mock_backend

def test_stress_collection(mock_backend_loop):
    loop, mock_backend = mock_backend_loop

    # Run a step
    metrics = loop.step({}, duration=0.1)

    # Current behavior: max_stress is hardcoded to 0.0
    # Expected behavior: max_stress should be 200.0 (max of 100 and 200)

    assert metrics.max_stress == 200.0
