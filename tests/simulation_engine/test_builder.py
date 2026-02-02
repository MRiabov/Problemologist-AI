import pytest
import trimesh
from build123d import Box

from simulation_engine.builder import MeshProcessor


def test_box_export_and_load():
    # Create a simple box
    box = Box(10, 10, 10)

    # Export to STL
    stl_data = MeshProcessor.export_stl(box)
    assert isinstance(stl_data, bytes)
    assert len(stl_data) > 0

    # Load mesh
    mesh = MeshProcessor.load_mesh(stl_data)
    assert isinstance(mesh, trimesh.Trimesh)
    assert not mesh.is_empty

    # Verify bounds (approximate)
    assert abs(mesh.extents[0] - 10) < 0.1
    assert abs(mesh.extents[1] - 10) < 0.1
    assert abs(mesh.extents[2] - 10) < 0.1


def test_convex_hull_complex_shape():
    # Create a complex (concave) shape: Two boxes joined at an angle
    b1 = Box(10, 2, 2)
    b2 = Box(2, 10, 2)
    shape = b1 + b2  # Union

    stl_data = MeshProcessor.export_stl(shape)
    mesh = MeshProcessor.load_mesh(stl_data)

    # The union of two perpendicular boxes is likely not convex
    # Note: build123d export might produce a single mesh that isn't strictly convex
    # but let's check if the convex hull works.

    hull = MeshProcessor.compute_convex_hull(mesh)
    assert isinstance(hull, trimesh.Trimesh)
    assert hull.is_convex
    assert hull.is_watertight
    assert not hull.is_empty


def test_load_mesh_from_empty_data():
    with pytest.raises(Exception):  # trimesh or io might raise different errors
        MeshProcessor.load_mesh(b"")
