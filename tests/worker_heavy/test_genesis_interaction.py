import numpy as np
import pytest

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]
from unittest.mock import MagicMock, patch

from shared.simulation.backends import SimulationScene
from worker_heavy.simulation.genesis_backend import GenesisBackend


@pytest.fixture
def genesis_backend():
    with patch("genesis.init"), patch("genesis.Scene"):
        backend = GenesisBackend()
        yield backend


def test_apply_control(genesis_backend):
    mock_entity = MagicMock()
    # Ensure it only has one of the control methods to avoid ambiguity
    del mock_entity.control_dofs_force
    genesis_backend.entities = {"motor1": mock_entity}
    genesis_backend.motors = [{"part_name": "motor1"}]

    genesis_backend.apply_control({"motor1": 10.0})

    # Check if set_dofs_force was called with a numpy array containing 10.0
    mock_entity.set_dofs_force.assert_called_once()
    args, kwargs = mock_entity.set_dofs_force.call_args
    assert np.allclose(args[0], np.array([10.0]))


def test_check_collision_with_zone(genesis_backend):
    # Mock scene_meta with a zone
    mock_scene = SimulationScene(scene_path="dummy.json")
    # For simplicity, we manually inject into GenesisBackend's logic
    genesis_backend.scene_meta = MagicMock()
    genesis_backend.scene_meta.assets = {
        "entities": [
            {"name": "zone_forbid", "is_zone": True, "min": [0, 0, 0], "max": [1, 1, 1]}
        ]
    }

    mock_body = MagicMock()
    mock_body.get_pos.return_value = np.array([0.5, 0.5, 0.5])
    mock_body.get_quat.return_value = np.array([1, 0, 0, 0])
    mock_body.get_vel.return_value = np.array([0, 0, 0])
    mock_body.get_ang.return_value = np.array([0, 0, 0])
    genesis_backend.entities = {"body1": mock_body}

    assert genesis_backend.check_collision("body1", "zone_forbid") is True

    mock_body.get_pos.return_value = np.array([2.0, 0.5, 0.5])
    assert genesis_backend.check_collision("body1", "zone_forbid") is False
