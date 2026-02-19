from unittest.mock import patch

import trimesh
from build123d import Box, Compound

from shared.models.schemas import (
    BoundingBox,
    Constraints,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
    PhysicsConfig,
)
from worker.simulation.builder import GenesisSimulationBuilder


def create_dummy_stl(part, file_path, *args, **kwargs):
    mesh = trimesh.creation.box(extents=[1, 1, 1])
    mesh.export(str(file_path))


@patch("worker.utils.mesh_utils.tetrahedralize")
@patch("worker.utils.mesh_utils.repair_mesh_file")
@patch("worker.simulation.builder.export_stl", side_effect=create_dummy_stl)
def test_genesis_builder_generates_msh_when_fem_enabled(
    mock_export_stl, mock_repair, mock_tetra, tmp_path
):
    # Setup
    output_dir = tmp_path / "output"
    builder = GenesisSimulationBuilder(output_dir)

    # Create a simple assembly
    from shared.models.schemas import PartMetadata

    box = Box(10, 10, 10)
    box.label = "test_part"
    box.metadata = PartMetadata(material_id="aluminum-6061")
    assembly = Compound(children=[box])

    # Define objectives with FEM enabled
    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
            build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        ),
        physics=PhysicsConfig(backend="genesis", fem_enabled=True),
        simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        moved_object=MovedObject(
            label="test_part",
            start_position=(0, 0, 0),
            runtime_jitter=(0, 0, 0),
            shape="box",
        ),
        constraints=Constraints(max_unit_cost=100, max_weight_g=10),
    )

    # Execute
    scene_path = builder.build_from_assembly(assembly, objectives=objectives)

    # Assert
    assert scene_path.exists()

    # Check if tetrahedralize was called
    assert mock_tetra.called
    assert mock_repair.called
    # It should be called at least twice: once by process_geometry (MuJoCo) and once for FEM
    # Actually GenesisSimulationBuilder calls process_geometry which calls export_stl
    # and then it calls export_stl AGAIN for FEM.
    assert mock_export_stl.call_count >= 2


@patch("worker.utils.mesh_utils.tetrahedralize")
@patch("worker.utils.mesh_utils.repair_mesh_file")
@patch("worker.simulation.builder.export_stl", side_effect=create_dummy_stl)
def test_genesis_builder_no_msh_when_fem_disabled(
    mock_export_stl, mock_repair, mock_tetra, tmp_path
):
    # Setup
    output_dir = tmp_path / "output"
    builder = GenesisSimulationBuilder(output_dir)

    # Create a simple assembly
    from shared.models.schemas import PartMetadata

    box = Box(10, 10, 10)
    box.label = "test_part"
    box.metadata = PartMetadata(material_id="aluminum-6061")
    assembly = Compound(children=[box])

    # Define objectives with FEM disabled
    objectives = ObjectivesYaml(
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(0, 0, 0), max=(1, 1, 1)),
            build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        ),
        physics=PhysicsConfig(backend="genesis", fem_enabled=False),
        simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
        moved_object=MovedObject(
            label="test_part",
            start_position=(0, 0, 0),
            runtime_jitter=(0, 0, 0),
            shape="box",
        ),
        constraints=Constraints(max_unit_cost=100, max_weight_g=10),
    )

    # Execute
    builder.build_from_assembly(assembly, objectives=objectives)

    # Assert
    assert not mock_tetra.called
    assert not mock_repair.called
