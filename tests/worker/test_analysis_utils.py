
import io
import trimesh
from build123d import Box
from worker.workbenches.analysis_utils import part_to_trimesh, compute_part_hash

def test_part_to_trimesh():
    """
    Verify that part_to_trimesh correctly converts a build123d Part to a Trimesh object.
    """
    part = Box(10, 10, 10)
    mesh = part_to_trimesh(part)
    assert isinstance(mesh, trimesh.Trimesh)
    assert len(mesh.vertices) > 0
    # A box has 12 triangles (2 per face * 6 faces)
    assert len(mesh.faces) == 12

def test_compute_part_hash():
    """
    Verify that compute_part_hash returns a stable hash string.
    """
    part = Box(10, 10, 10)
    hash1 = compute_part_hash(part)
    assert isinstance(hash1, str)
    assert len(hash1) > 0

    # Check stability (same part geometry -> same hash)
    part2 = Box(10, 10, 10)
    hash2 = compute_part_hash(part2)
    assert hash1 == hash2

    # Check difference (different part geometry -> different hash)
    part3 = Box(20, 10, 10)
    hash3 = compute_part_hash(part3)
    assert hash1 != hash3
