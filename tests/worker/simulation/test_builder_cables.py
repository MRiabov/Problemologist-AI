
import unittest
from unittest.mock import MagicMock, patch
import json
from pathlib import Path
import tempfile
import sys

# Mock modules that might be missing or problematic
sys.modules["genesis"] = MagicMock()

# Import after mocking
from worker.simulation.builder import GenesisSimulationBuilder, AssemblyPartData

class TestGenesisBuilderCables(unittest.TestCase):
    @patch("worker.simulation.builder.CommonAssemblyTraverser")
    @patch("worker.simulation.builder.export_stl")
    @patch("worker.simulation.builder.MeshProcessor")
    @patch("worker.workbenches.config.load_config")
    def test_cables_generation(self, mock_load_config, mock_processor, mock_export, mock_traverser):
        # Setup mocks
        mock_processor_instance = mock_processor.return_value
        mock_processor_instance.process_geometry.return_value = []

        # Mock mfg config
        mock_config = MagicMock()
        mock_config.materials.get.return_value = None
        mock_load_config.return_value = mock_config

        # Mock parts data
        part1 = AssemblyPartData(
            label="part1", part=MagicMock(), pos=[0, 0, 0], euler=[0, 0, 0],
            is_fixed=True, material_id="alu", cots_id="cots1",
            is_electronics=True, is_zone=False
        )
        part2 = AssemblyPartData(
            label="part2", part=MagicMock(), pos=[10, 0, 0], euler=[0, 0, 0],
            is_fixed=True, material_id="alu", cots_id="cots2",
            is_electronics=True, is_zone=False
        )
        mock_traverser.traverse.return_value = [part1, part2]

        # Mock electronics
        mock_wire1 = MagicMock()
        mock_wire1.wire_id = "wire1"
        mock_wire1.gauge_awg = 20
        mock_wire1.routed_in_3d = True
        mock_wire1.waypoints = [[0, 0, 0], [5, 0, 0], [10, 0, 0]]

        mock_wire2 = MagicMock()
        mock_wire2.wire_id = "wire2"
        mock_wire2.gauge_awg = 10
        mock_wire2.routed_in_3d = True
        mock_wire2.waypoints = [] # No waypoints, fallback
        mock_wire2.from_terminal.component = "part1"
        mock_wire2.to_terminal.component = "part2"

        mock_electronics = MagicMock()
        mock_electronics.wiring = [mock_wire1, mock_wire2]

        # Run builder
        with tempfile.TemporaryDirectory() as tmpdir:
            builder = GenesisSimulationBuilder(output_dir=tmpdir)
            scene_path = builder.build_from_assembly(
                assembly=MagicMock(),
                electronics=mock_electronics
            )

            # Verify output
            with open(scene_path) as f:
                data = json.load(f)

            self.assertIn("cables", data)
            cables = data["cables"]
            self.assertEqual(len(cables), 2)

            # Check wire1
            w1 = next(c for c in cables if c["name"] == "wire1")
            self.assertEqual(w1["points"], [[0, 0, 0], [5, 0, 0], [10, 0, 0]])
            # AWG 20 radius: (0.001 + (20-20)*0.0001)/2 = 0.0005
            self.assertAlmostEqual(w1["radius"], 0.0005)

            # Check wire2
            w2 = next(c for c in cables if c["name"] == "wire2")
            # Should connect part1 (0,0,0) and part2 (10,0,0)
            self.assertEqual(w2["points"], [[0, 0, 0], [10, 0, 0]])
            # AWG 10 radius: (0.001 + (20-10)*0.0001)/2 = (0.001 + 0.001)/2 = 0.001
            self.assertAlmostEqual(w2["radius"], 0.001)

if __name__ == "__main__":
    unittest.main()
