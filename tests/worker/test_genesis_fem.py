import numpy as np
import pytest

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]
from unittest.mock import MagicMock, patch

from shared.simulation.backends import SimulationScene
from worker_heavy.simulation.genesis_backend import GenesisBackend


@pytest.fixture
def mock_gs():
    with patch("worker_heavy.simulation.genesis_backend.gs") as mock:
        yield mock


def test_genesis_load_scene_soft_mesh(mock_gs, tmp_path):
    backend = GenesisBackend()

    # Create dummy scene.json
    scene_json = tmp_path / "scene.json"
    import json

    scene_data = {
        "entities": [
            {
                "name": "soft_part",
                "type": "soft_mesh",
                "file": "part.msh",
                "pos": [0, 0, 0],
                "euler": [0, 0, 0],
                "material_id": "silicone_rubber",
            }
        ]
    }
    scene_json.write_text(json.dumps(scene_data))

    scene = SimulationScene(scene_path=str(scene_json))

    # Mock mfg_config
    backend.mfg_config = MagicMock()
    mock_mat = MagicMock()
    mock_mat.material_class = "elastomer"
    mock_mat.youngs_modulus_pa = 5e6
    mock_mat.poissons_ratio = 0.49
    mock_mat.density_kg_m3 = 1100
    backend.mfg_config.materials = {"silicone_rubber": mock_mat}

    backend.load_scene(scene)

    assert "soft_part" in backend.entities
    # Verify NeoHookean was used for elastomer
    assert mock_gs.materials.FEM.NeoHookean.called


def test_genesis_step_breakage(mock_gs):
    backend = GenesisBackend()
    backend.scene = MagicMock()
    backend.scene.is_built = True

    # Setup entity
    mock_entity = MagicMock()
    backend.entities = {"test_part": mock_entity}

    # Mock stress field
    mock_state = MagicMock()
    mock_state.von_mises = [MagicMock()]
    mock_state.von_mises[0].cpu().numpy.return_value = np.array(
        [400e6]
    )  # Exceeds 310e6
    mock_state.pos = [MagicMock()]
    mock_state.pos[0].cpu().numpy.return_value = np.array([[0, 0, 0]])
    mock_entity.get_state.return_value = mock_state

    res = backend.step(0.002)

    assert res.success is False
    assert "PART_BREAKAGE:test_part" in res.failure_reason


def test_get_stress_summaries(mock_gs):
    backend = GenesisBackend()

    # Setup entity with stress
    mock_entity = MagicMock()
    backend.entities = {"part1": mock_entity}

    mock_state = MagicMock()
    mock_state.von_mises = [MagicMock()]
    mock_state.von_mises[0].cpu().numpy.return_value = np.array([100e6, 200e6])
    mock_state.pos = [MagicMock()]
    mock_state.pos[0].cpu().numpy.return_value = np.array([[0, 0, 0], [1, 1, 1]])
    mock_entity.get_state.return_value = mock_state

    summaries = backend.get_stress_summaries()

    assert len(summaries) == 1
    assert summaries[0].part_label == "part1"
    assert summaries[0].max_von_mises_pa == 200e6
