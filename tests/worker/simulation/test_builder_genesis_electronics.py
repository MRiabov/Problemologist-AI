
import json
import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path
import sys

# Mock imports that might be missing or problematic
sys.modules["worker.workbenches.config"] = MagicMock()

from worker.simulation.builder import GenesisSimulationBuilder

# Mock build123d objects
class MockSolid:
    def __init__(self, label="part"):
        self.label = label
        self.metadata = {"material_id": "aluminum", "is_fixed": True}
        class Location:
            class Vector:
                def __init__(self, x, y, z):
                    self.X, self.Y, self.Z = x, y, z
            position = Vector(0, 0, 0)
            orientation = Vector(0, 0, 0)
        self.location = Location()
        self.children = []

    def bounding_box(self):
        mock_bb = MagicMock()
        mock_bb.size.X = 1
        mock_bb.size.Y = 1
        mock_bb.size.Z = 1
        return mock_bb

    def faces(self):
        return []

class MockCompound:
    def __init__(self, children):
        self.children = children
        self.label = "assembly"
        class Location:
            class Vector:
                def __init__(self, x, y, z):
                    self.X, self.Y, self.Z = x, y, z
            position = Vector(0, 0, 0)
            orientation = Vector(0, 0, 0)
        self.location = Location()

@pytest.fixture
def mock_electronics():
    mock_wire = MagicMock()
    mock_wire.wire_id = "cable_1"
    mock_wire.gauge_awg = 20
    mock_wire.routed_in_3d = True
    mock_wire.waypoints = [(0.1, 0.1, 0.1), (0.2, 0.2, 0.2)]
    mock_wire.from_terminal.component = "part_1"
    mock_wire.to_terminal.component = "part_2"

    mock_electronics = MagicMock()
    mock_electronics.wiring = [mock_wire]
    return mock_electronics

@patch("worker.simulation.builder.MeshProcessor")
@patch("worker.simulation.builder.export_stl")
@patch("worker.workbenches.config.load_config")
def test_genesis_builder_electronics(mock_load_config, mock_export, mock_processor, tmp_path, mock_electronics):
    # Setup mocks
    mock_config = MagicMock()
    mock_config.materials = {}
    mock_load_config.return_value = mock_config

    builder = GenesisSimulationBuilder(tmp_path)

    part = MockSolid("part_1")
    assembly = MockCompound([part])

    scene_path = builder.build_from_assembly(assembly, electronics=mock_electronics)

    assert scene_path.exists()

    with open(scene_path) as f:
        data = json.load(f)

    assert "cables" in data, "Cables list missing in scene.json"
    assert len(data["cables"]) == 1
    cable = data["cables"][0]
    assert cable["name"] == "cable_1"
    assert len(cable["points"]) == 2
    assert "radius" in cable
    assert "stiffness" in cable
