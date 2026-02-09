import hashlib
import pathlib
import tempfile

import numpy as np
import trimesh
from build123d import Compound, Part, Solid


def part_to_trimesh(part: Part | Compound | Solid) -> trimesh.Trimesh:
    """
    Converts a build123d Part, Solid, or Compound to a trimesh.Trimesh object.
    """
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        from build123d import export_stl

        export_stl(part, tmp_path)
        mesh = trimesh.load(tmp_path)
        # trimesh.load can return a Scene, we want a single mesh
        if isinstance(mesh, trimesh.Scene):
            mesh = mesh.dump(concatenate=True)
        return mesh
    finally:
        if pathlib.Path(tmp_path).exists():
            pathlib.Path(tmp_path).unlink()


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
        # Optimize: vectorized check for base faces
        # Get Z coordinates of all vertices in candidate faces
        # mesh.vertices is (V, 3), so we take Z column first -> (V,)
        # mesh.faces[pointing_away] is (N, 3) indices
        # Result is (N, 3) array of Z coordinates
        face_z = mesh.vertices[:, 2][mesh.faces[pointing_away]]

        # Check if all vertices of a face are at min_z
        is_at_bottom = np.all(np.abs(face_z - min_z) < 0.01, axis=1)

        # Check if normal is pointing straight down (nearly -1 dot product)
        is_pointing_down = dots[pointing_away] < -0.99

        # Base faces are those at the bottom AND pointing down
        is_base = is_at_bottom & is_pointing_down

        # Keep faces that are NOT base faces
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

        intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
        hits = intersector.intersects_any(origins, directions)

        occluded_indices = pointing_towards[hits]
        undercut_indices.extend(occluded_indices.tolist())

    return list(set(undercut_indices))


def compute_part_hash(part: Part | Compound | Solid) -> str:
    """
    Computes a stable hash for a build123d Part, Solid, or Compound.
    Uses STL export as a proxy for geometry.
    """
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        from build123d import export_stl

        export_stl(part, tmp_path)
        with open(tmp_path, "rb") as f:
            content = f.read()
        return hashlib.sha256(content).hexdigest()
    finally:
        if pathlib.Path(tmp_path).exists():
            pathlib.Path(tmp_path).unlink()
