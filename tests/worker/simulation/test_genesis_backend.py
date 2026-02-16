import sys
from unittest.mock import MagicMock

# Mock genesis module before importing GenesisBackend
mock_genesis = MagicMock()
sys.modules["genesis"] = mock_genesis
# Set attributes that might be accessed
mock_genesis.gpu = "gpu"
mock_genesis.cpu = "cpu"

from worker.simulation.genesis_backend import GenesisBackend
from shared.simulation.backends import SimulationScene

def test_load_cables(tmp_path):
    # Create a scene.json with cables
    scene_path = tmp_path / "scene.json"
    import json
    data = {
        "entities": [],
        "motors": [],
        "fluids": [],
        "objectives": [],
        "cables": [
            {
                "name": "wire1",
                "radius": 0.001,
                "points": [{"type": "world", "pos": [0,0,0]}, {"type": "world", "pos": [1,1,1]}]
            }
        ]
    }
    with open(scene_path, "w") as f:
        json.dump(data, f)

    backend = GenesisBackend()
    scene = SimulationScene(scene_path=str(scene_path))

    # load_scene checks if gs is None.
    # Since we mocked sys.modules["genesis"], the import inside genesis_backend.py
    # (which might have already happened if imported elsewhere, but pytest re-imports or handles it)
    # The try-except block in genesis_backend.py is at module level.
    # If it was already imported as None, we are in trouble.
    # But since we set sys.modules["genesis"] before importing here, and hopefully before other imports in this process (unlikely if pytest loads all),
    # we might need to reload the module.

    import worker.simulation.genesis_backend
    import importlib
    importlib.reload(worker.simulation.genesis_backend)

    backend = worker.simulation.genesis_backend.GenesisBackend()
    backend.load_scene(scene)

    assert len(backend.cables) == 1
    assert backend.cables[0]["name"] == "wire1"
