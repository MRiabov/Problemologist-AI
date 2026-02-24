from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from shared.models.schemas import ObjectivesYaml
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.simulation.loop import SimulationLoop


@pytest.mark.unit
def test_fluid_containment_logic(tmp_path):
    """
    Verify that fluid particles are correctly counted against zones in SimulationLoop.
    """
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")

    objectives_dict = {
        "objectives": {
            "goal_zone": {"min": [10, 10, 10], "max": [12, 12, 12]},
            "build_zone": {"min": [-5, -5, -5], "max": [5, 5, 5]},
            "fluid_objectives": [
                {
                    "type": "fluid_containment",
                    "fluid_id": "water",
                    "containment_zone": {"min": [-1, -1, -1], "max": [1, 1, 1]},
                    "threshold": 0.9,
                    "eval_at": "end",
                }
            ],
        },
        "simulation_bounds": {"min": [-20, -20, -20], "max": [20, 20, 20]},
        "moved_object": {
            "label": "obj",
            "shape": "sphere",
            "start_position": [0, 0, 0],
            "runtime_jitter": [0, 0, 0],
        },
        "constraints": {"max_unit_cost": 100, "max_weight_g": 10},
    }

    with patch("worker_heavy.simulation.loop.get_physics_backend") as mock_get_backend:
        mock_backend = MagicMock()
        mock_get_backend.return_value = mock_backend
        # Numeric values to avoid max(mock, float) errors
        mock_backend.step.return_value = MagicMock(
            time=0.1, success=True, energy=0.0, velocity=0.0, stress=0.0
        )
        mock_backend.get_max_stress.return_value = 0.0
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_body_names.return_value = []
        mock_backend.get_all_site_names.return_value = []

        # Mock particles: 100 particles, 95 inside the zone
        particles = np.zeros((100, 3))
        # 95 at (0,0,0) - inside [-1, 1]
        # 5 at (5,5,5) - outside
        particles[95:] = [5, 5, 5]
        mock_backend.get_particle_positions.return_value = particles

        loop = SimulationLoop(
            str(xml_path),
            backend_type=SimulatorBackendType.GENESIS,
            objectives=ObjectivesYaml(**objectives_dict),
        )

        metrics = loop.step(control_inputs={}, duration=0.1)

        # 95/100 = 0.95 >= 0.9 threshold -> SUCCESS
        assert len(metrics.fluid_metrics) == 1
        assert metrics.fluid_metrics[0].passed
        assert metrics.fluid_metrics[0].measured_value == 0.95

        # Fail case
        particles[:] = [5, 5, 5]  # All outside
        mock_backend.get_particle_positions.return_value = particles

        metrics_fail = loop.step(control_inputs={}, duration=0.1)
        assert not metrics_fail.fluid_metrics[0].passed
        assert metrics_fail.fluid_metrics[0].measured_value == 0.0
