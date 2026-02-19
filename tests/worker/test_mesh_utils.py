from pathlib import Path
from unittest.mock import patch

import pytest
import trimesh

from worker_heavy.utils.mesh_utils import MeshProcessingError, repair_mesh, tetrahedralize


def test_repair_mesh_already_watertight():
    # Create a simple box which is watertight
    mesh = trimesh.creation.box(extents=[1, 1, 1])
    assert mesh.is_watertight

    repaired = repair_mesh(mesh)
    assert repaired == mesh
    assert repaired.is_watertight


def test_repair_mesh_fixes_inverted_normals():
    # Create a box and invert some normals
    mesh = trimesh.creation.box(extents=[1, 1, 1])
    mesh.faces = mesh.faces[:, [0, 2, 1]]  # Invert winding

    # We don't need to mock watertightness check if we use a real mesh
    # and trust trimesh.fix_normals()
    repaired = repair_mesh(mesh)
    assert repaired.is_watertight


def test_repair_mesh_fails_after_max_attempts():
    # Create a non-watertight mesh that cannot be easily fixed
    # A single triangle is not watertight
    mesh = trimesh.Trimesh(
        vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], faces=[[0, 1, 2]]
    )
    assert not mesh.is_watertight

    with pytest.raises(MeshProcessingError, match="Mesh failed to become watertight"):
        repair_mesh(mesh, max_attempts=1)


@patch("gmsh.finalize")
@patch("gmsh.write")
@patch("gmsh.model.mesh.getElements")
@patch("gmsh.model.mesh.optimize")
@patch("gmsh.model.mesh.generate")
@patch("gmsh.option.setNumber")
@patch("gmsh.model.geo.synchronize")
@patch("gmsh.model.geo.addVolume")
@patch("gmsh.model.geo.addSurfaceLoop")
@patch("gmsh.model.getEntities")
@patch("gmsh.merge")
@patch("gmsh.model.add")
@patch("gmsh.initialize")
@patch("gmsh.isInitialized")
def test_tetrahedralize_gmsh_success(
    mock_is_init,
    mock_init,
    mock_model_add,
    mock_merge,
    mock_get_entities,
    mock_add_loop,
    mock_add_vol,
    mock_sync,
    mock_set_num,
    mock_generate,
    mock_optimize,
    mock_get_elements,
    mock_write,
    mock_finalize,
    tmp_path,
):
    mock_is_init.side_effect = [
        False,
        True,
    ]  # First for initialize check, second for finalize check
    mock_get_entities.return_value = [(2, 1)]
    mock_get_elements.return_value = ([4], None, None)  # 4 is the type for tetrahedrons

    input_stl = tmp_path / "input.stl"
    input_stl.write_text("solid test\nendsolid test")
    output_msh = tmp_path / "output.msh"

    result = tetrahedralize(input_stl, output_msh, method="gmsh")

    assert result == output_msh
    mock_init.assert_called_once()
    mock_merge.assert_called_once_with(str(input_stl))
    mock_generate.assert_called_once_with(3)
    mock_write.assert_called_once_with(str(output_msh))
    mock_finalize.assert_called_once()


def test_tetrahedralize_file_not_found():
    with pytest.raises(FileNotFoundError):
        tetrahedralize(Path("nonexistent.stl"), Path("out.msh"))


def test_tetrahedralize_invalid_method():
    with pytest.raises(
        MeshProcessingError,
        match="Tetrahedralization failed: Unknown tetrahedralization method",
    ):
        tetrahedralize(Path(), Path(), method="invalid")
