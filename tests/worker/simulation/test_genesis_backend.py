
import sys
import unittest
from unittest.mock import MagicMock, patch
import json
import numpy as np

# Mock genesis module
sys.modules["genesis"] = MagicMock()
import genesis as gs
gs.gpu = "gpu"
gs.cpu = "cpu"
# Mock Scene and Entity
mock_scene = MagicMock()
mock_scene.add_entity = MagicMock()
mock_scene.add_camera = MagicMock()
gs.Scene = MagicMock(return_value=mock_scene)
gs.morphs = MagicMock()
gs.materials = MagicMock()
gs.options = MagicMock()

from worker.simulation.genesis_backend import GenesisBackend
from shared.simulation.backends import SimulationScene

class TestGenesisBackend(unittest.TestCase):
    def setUp(self):
        self.backend = GenesisBackend()
        self.backend.mfg_config = MagicMock() # Mock config

    def test_cables_loading(self):
        # Create a dummy scene json with cables
        scene_data = {
            "entities": [
                {"name": "body1", "file": "body1.obj", "pos": [0,0,0], "euler": [0,0,0], "type": "rigid"}
            ],
            "cables": [
                {
                    "name": "cable1",
                    "points": [[0,0,0], [1,1,1]],
                    "radius": 0.001,
                    "stiffness": 100
                }
            ]
        }

        with patch("pathlib.Path.open") as mock_open:
            mock_file = MagicMock()
            mock_file.__enter__.return_value = mock_file
            mock_file.read.return_value = json.dumps(scene_data)
            with patch("json.load", return_value=scene_data):
                scene = SimulationScene(scene_path="dummy.json")
                self.backend.load_scene(scene)

        # Check if cables are loaded
        tendons = self.backend.get_all_tendon_names()
        self.assertIn("cable1", tendons)

        # Check if cable state can be queried (even if dummy)
        # For now, just existence is enough as tension might require sim

    def test_zones_loading(self):
        # Create a dummy scene json with a zone
        scene_data = {
            "entities": [
                {"name": "zone1", "is_zone": True, "pos": [1,1,1], "size": [0.5, 0.5, 0.5], "type": "box"}
            ]
        }

        with patch("pathlib.Path.open") as mock_open:
            mock_file = MagicMock()
            mock_file.__enter__.return_value = mock_file
            mock_file.read.return_value = json.dumps(scene_data)
            with patch("json.load", return_value=scene_data):
                scene = SimulationScene(scene_path="dummy.json")
                self.backend.load_scene(scene)

        # Check if zone is recognized as a site
        sites = self.backend.get_all_site_names()
        self.assertIn("zone1", sites)

        # Check get_site_state
        state = self.backend.get_site_state("zone1")
        self.assertEqual(state.pos, (1, 1, 1))

        # Check collision with zone
        with patch.object(self.backend, "get_body_state") as mock_get_state:
             mock_get_state.return_value = MagicMock(pos=[1,1,1])
             collision = self.backend.check_collision("some_body", "zone1")
             self.assertTrue(collision)

    def test_set_site_pos(self):
        # Create a dummy scene json with a zone
        scene_data = {
            "entities": [
                {"name": "zone1", "is_zone": True, "pos": [1,1,1], "size": [0.5, 0.5, 0.5], "type": "box"}
            ]
        }

        with patch("pathlib.Path.open") as mock_open:
            mock_file = MagicMock()
            mock_file.__enter__.return_value = mock_file
            mock_file.read.return_value = json.dumps(scene_data)
            with patch("json.load", return_value=scene_data):
                scene = SimulationScene(scene_path="dummy.json")
                self.backend.load_scene(scene)

        new_pos = np.array([2.0, 2.0, 2.0])
        self.backend.set_site_pos("zone1", new_pos)

        state = self.backend.get_site_state("zone1")
        self.assertEqual(state.pos, (2.0, 2.0, 2.0))

if __name__ == "__main__":
    unittest.main()
