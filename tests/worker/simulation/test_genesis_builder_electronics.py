import json
import unittest
from unittest.mock import MagicMock, patch
from pathlib import Path
import tempfile
import sys
import os

from worker_heavy.simulation.builder import GenesisSimulationBuilder
from worker_heavy.simulation.genesis_backend import GenesisBackend
import worker_heavy.workbenches.config
from shared.models.schemas import WireConfig, WireTerminal


class TestGenesisBuilderElectronics(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.output_dir = Path(self.temp_dir.name)
        self.builder = GenesisSimulationBuilder(self.output_dir)

    def tearDown(self):
        self.temp_dir.cleanup()

    def test_cables_in_scene_json(self):
        # Mock assembly
        mock_assembly = MagicMock()
        mock_assembly.children = []

        # Mock electronics with one wire
        mock_wire = MagicMock()
        mock_wire.wire_id = "wire_1"
        mock_wire.gauge_awg = 20
        mock_wire.routed_in_3d = True
        mock_wire.waypoints = [(0, 0, 0), (1, 1, 1)]
        mock_wire.from_terminal = WireTerminal(component="comp1", terminal="t1")
        mock_wire.to_terminal = WireTerminal(component="comp2", terminal="t2")

        mock_electronics = MagicMock()
        mock_electronics.wiring = [mock_wire]
        mock_electronics.components = []

        # Mock traverse to avoid complex logic
        with patch(
            "worker_heavy.simulation.builder.CommonAssemblyTraverser.traverse",
            return_value=[],
        ):
            # Also patch load_config to avoid config loading issues
            with patch("worker_heavy.workbenches.config.load_config"):
                self.builder.build_from_assembly(
                    assembly=mock_assembly, electronics=mock_electronics
                )

        scene_path = self.output_dir / "scene.json"
        self.assertTrue(scene_path.exists())

        with open(scene_path, "r") as f:
            data = json.load(f)

        self.assertIn("cables", data, "'cables' key missing in scene.json")
        self.assertEqual(len(data["cables"]), 1)
        self.assertEqual(data["cables"][0]["name"], "wire_1")
        self.assertEqual(len(data["cables"][0]["points"]), 2)

    def test_applied_control_persistence(self):
        backend = GenesisBackend()
        # Mock entity
        mock_entity = MagicMock()
        backend.entities = {"motor1": mock_entity}
        backend.motors = [{"part_name": "motor1"}]

        # Apply control
        backend.apply_control({"motor1": 0.5})

        # Check state
        state = backend.get_actuator_state("motor1")

        self.assertEqual(state.ctrl, 0.5, f"Expected ctrl 0.5, got {state.ctrl}")

    def test_tendon_support(self):
        backend = GenesisBackend()
        # Inject cables manually as if loaded from scene
        # It should be a dict mapping name to entity
        backend.cables = {
            "wire_1": MagicMock()
        }

        self.assertTrue(
            hasattr(backend, "cables"), "GenesisBackend has no 'cables' attribute"
        )

        # Check get_all_tendon_names
        names = backend.get_all_tendon_names()
        self.assertIsInstance(names, list)
        self.assertIn("wire_1", names)

        # Check get_tendon_tension
        # Mocking the tendon tension logic which uses waypoint distances
        backend.tendon_rest_lengths = {"wire_1": 1.0}
        backend.tendon_waypoints = {"wire_1": [(0,0,0), (1,0,0)]}

        # We need to mock gs if we want it to actually run distance calc,
        # or just mock get_tendon_tension if we can.
        # But get_tendon_tension is what we are testing (or at least its existence).

        # In GenesisBackend, get_tendon_tension calculates Euclidean distance between global waypoints.
        # It needs self.entities and self.tendon_waypoints

        tension = backend.get_tendon_tension("wire_1")
        self.assertEqual(tension, 0.0)

        with self.assertRaises(ValueError):
            backend.get_tendon_tension("non_existent_wire")


if __name__ == "__main__":
    unittest.main()
