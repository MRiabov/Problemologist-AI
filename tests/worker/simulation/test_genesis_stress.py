import sys
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from shared.simulation.backends import StepResult, StressField
from worker.simulation.genesis_backend import GenesisBackend
from worker.simulation.loop import SimulationLoop

# Mock genesis module structure
mock_gs = MagicMock()
mock_gs.gpu = "gpu"
mock_gs.cpu = "cpu"
mock_gs.init = MagicMock()

@pytest.fixture
def backend():
    # Patch 'gs' in the module to avoid ImportError or None
    with patch("worker.simulation.genesis_backend.gs", mock_gs):
        backend = GenesisBackend()
        # Setup a mock scene
        backend.scene = MagicMock()
        backend.scene.is_built = True
        return backend

def test_genesis_backend_step_returns_max_stress(backend):
    # Setup entities
    backend.entities = {"part1": MagicMock(), "part2": MagicMock()}

    # Mock get_stress_field to return different stress values
    def get_stress_side_effect(name):
        if name == "part1":
            # Max stress 200.0
            return StressField(nodes=np.zeros((2,3)), stress=np.array([100.0, 200.0]))
        elif name == "part2":
            # Max stress 300.0
            return StressField(nodes=np.zeros((2,3)), stress=np.array([50.0, 300.0]))
        return None

    # Patch get_stress_field on the instance
    with patch.object(backend, "get_stress_field", side_effect=get_stress_side_effect):
        # Also need to mock _check_electronics_fluid_damage to avoid errors
        with patch.object(backend, "_check_electronics_fluid_damage", return_value=None):
            res = backend.step(0.01)

            assert res.success is True
            # Should be max of 200.0 and 300.0
            assert res.max_stress == 300.0

def test_simulation_loop_collects_max_stress(tmp_path):
    # Create a dummy XML file
    xml_path = tmp_path / "scene.xml"
    xml_path.write_text("<mujoco/>")

    # Mock get_physics_backend to return our mock backend
    with patch("worker.simulation.loop.get_physics_backend") as mock_get_backend:
        mock_backend = MagicMock()
        # Configure step return value with max_stress
        mock_backend.step.return_value = StepResult(time=0.1, success=True, max_stress=1234.5)

        # Mock other required methods for SimulationLoop init and step
        mock_backend.get_all_body_names.return_value = ["body1"]
        mock_backend.get_all_actuator_names.return_value = []
        mock_backend.get_all_site_names.return_value = []
        mock_backend.get_actuator_state.return_value = MagicMock(force=0, velocity=0, ctrl=0)
        mock_backend.get_body_state.return_value = MagicMock(pos=np.zeros(3), vel=np.zeros(3))
        mock_backend.get_stress_summaries.return_value = []
        mock_backend.get_particle_positions.return_value = None
        mock_backend.check_collision.return_value = False

        mock_get_backend.return_value = mock_backend

        # Initialize loop
        loop = SimulationLoop(str(xml_path), backend_type="genesis")

        # Run one step
        loop.step({}, duration=0.01)

        # Check metrics
        metrics = loop.metric_collector.get_metrics()
        assert metrics.max_stress == 1234.5
