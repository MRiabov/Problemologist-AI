import hashlib
import io
import os
import pathlib
import tempfile

import numpy as np
import trimesh
from build123d import Compound, Part


def _get_stl_bytes(part: Part | Compound) -> bytes:
    """
    Exports a build123d Part or Compound to STL bytes.
    """
    # Use /dev/shm if available (Linux RAM disk) for faster I/O
    temp_dir = "/dev/shm" if os.path.exists("/dev/shm") else None
    with tempfile.NamedTemporaryFile(suffix=".stl", dir=temp_dir, delete=False) as tmp:
        tmp_path = tmp.name

    try:
        from build123d import export_stl

        export_stl(part, tmp_path)
        with open(tmp_path, "rb") as f:
            content = f.read()
        return content
    finally:
        if pathlib.Path(tmp_path).exists():
            pathlib.Path(tmp_path).unlink()


def analyze_geometry(part: Part | Compound) -> tuple[trimesh.Trimesh, str]:
    """
    Computes both the trimesh geometry and the part hash in one pass.
    Avoids double export to STL.
    """
    content = _get_stl_bytes(part)
    part_hash = hashlib.sha256(content).hexdigest()

    mesh = trimesh.load(io.BytesIO(content), file_type="stl")
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    return mesh, part_hash


def part_to_trimesh(part: Part | Compound) -> trimesh.Trimesh:
    """
    Converts a build123d Part or Compound to a trimesh.Trimesh object.
    """
    mesh, _ = analyze_geometry(part)
    return mesh


def check_undercuts(
    mesh: trimesh.Trimesh,
    approach_direction: tuple[float, float, float] = (0.0, 0.0, 1.0),
) -> list[int]:
    """
    Identifies faces that are undercuts from a given approach direction.
    A face is an undercut if its normal points away from the approach direction.

    Args:
        mesh: The trimesh.Trimesh to check.
        approach_direction: The vector pointing towards the tool (e.g., (0,0,1) for +Z approach).

    Returns:
        A list of face indices that are undercuts.
    """
    approach_direction = np.array(approach_direction)
    approach_direction = approach_direction / np.linalg.norm(approach_direction)

    # Dot product of face normals with approach direction
    dots = np.dot(mesh.face_normals, approach_direction)

    # 1. Faces pointing away (dot < -0.01)
    # Filter out faces that are at the very bottom of the part and point exactly down
    min_z = mesh.vertices[:, 2].min()
    pointing_away = np.where(dots < -0.01)[0]

    if len(pointing_away) > 0:
        # Get vertices for these faces: (N, 3, 3)
        faces_vertices = mesh.vertices[mesh.faces[pointing_away]]
        # Get Z coordinates: (N, 3)
        faces_z = faces_vertices[:, :, 2]

        # Check if all vertices of a face are at min_z
        is_at_bottom = np.all(np.abs(faces_z - min_z) < 0.01, axis=1)

        # Check if normal is pointing effectively straight down (opposite to Z if approach is Z)
        is_pointing_down = dots[pointing_away] < -0.99

        # Identify base faces
        is_base = is_at_bottom & is_pointing_down

        # Keep only non-base faces
        undercut_indices = pointing_away[~is_base].tolist()
    else:
        undercut_indices = []

    # 2. Occlusion check using raycasting
    # Faces that point TOWARDS the tool but are blocked by other geometry
    pointing_towards = np.where(dots >= -0.01)[0]
    if len(pointing_towards) > 0:
        centers = mesh.triangles_center[pointing_towards]
        # Offset centers slightly in the NORMAL direction to avoid self-intersection
        normals = mesh.face_normals[pointing_towards]
        origins = centers + normals * 1e-4
        directions = np.tile(approach_direction, (len(origins), 1))

        # Use mesh.ray for raycasting, which automatically uses pyembree if available (faster)
        # and caches the BVH structure on the mesh object for subsequent calls.
        hits = mesh.ray.intersects_any(origins, directions)

        occluded_indices = pointing_towards[hits]
        undercut_indices.extend(occluded_indices.tolist())

    return list(set(undercut_indices))


def compute_part_hash(part: Part | Compound) -> str:
    """
    Computes a stable hash for a build123d Part or Compound.
    Uses STL export as a proxy for geometry.
    """
    content = _get_stl_bytes(part)
    return hashlib.sha256(content).hexdigest()
