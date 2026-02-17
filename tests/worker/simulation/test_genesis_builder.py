
import json
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from build123d import Box, Compound
from shared.models.schemas import (
    ElectronicsSection,
    WireConfig,
    WireTerminal,
    PowerSupplyConfig,
    PartMetadata,
)
from worker.simulation.builder import GenesisSimulationBuilder

class TestGenesisBuilder(unittest.TestCase):
    def setUp(self):
        self.tmp_path = Path("test_output")
        self.tmp_path.mkdir(exist_ok=True)
        self.builder = GenesisSimulationBuilder(self.tmp_path)

    def tearDown(self):
        import shutil
        if self.tmp_path.exists():
            shutil.rmtree(self.tmp_path)

    def test_build_with_electronics(self):
        # Create a simple assembly
        box1 = Box(0.1, 0.1, 0.1)
        box1.label = "part1"
        box1.metadata = PartMetadata(material_id="aluminum_6061")

        box2 = Box(0.1, 0.1, 0.1)
        box2.label = "part2"
        box2.metadata = PartMetadata(material_id="aluminum_6061")

        assembly = Compound(children=[box1, box2])

        # Create electronics definition
        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(
                voltage_dc=12.0,
                max_current_a=5.0
            ),
            wiring=[
                WireConfig(
                    wire_id="wire1",
                    from_terminal=WireTerminal(component="part1", terminal="VCC"),
                    to_terminal=WireTerminal(component="part2", terminal="GND"),
                    gauge_awg=24,
                    length_mm=100.0,
                    routed_in_3d=True,
                    waypoints=[(0,0,0), (1,1,1)]
                )
            ]
        )

        # Mock processor to avoid actual mesh processing
        self.builder.processor = MagicMock()

        # Mock config loading
        with patch("worker.workbenches.config.load_config") as mock_load_config:
            mock_config = MagicMock()
            mock_config.materials = {}
            mock_load_config.return_value = mock_config

            scene_path = self.builder.build_from_assembly(assembly, electronics=electronics)

        # Read the generated scene.json
        with open(scene_path) as f:
            scene_data = json.load(f)

        # Assertions
        self.assertIn("cables", scene_data, "Scene JSON should contain 'cables' key")
        self.assertEqual(len(scene_data["cables"]), 1)
        cable = scene_data["cables"][0]
        self.assertEqual(cable["name"], "wire1")
        self.assertEqual(cable["gauge_awg"], 24)
        self.assertEqual(cable["points"], [[0,0,0], [1,1,1]])
        # Check attachment points if we implement that logic
        # For now, just checking the list presence and basic data

if __name__ == "__main__":
    unittest.main()
