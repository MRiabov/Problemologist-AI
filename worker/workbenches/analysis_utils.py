import hashlib
import pathlib
import tempfile

import numpy as np
import structlog
import trimesh
from build123d import Compound, Part

logger = structlog.get_logger()


def part_to_trimesh(part: Part | Compound) -> trimesh.Trimesh:
    """
    Converts a build123d Part or Compound to a trimesh.Trimesh object.
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

    undercut_indices = []
    for idx in pointing_away:
        face_vertices = mesh.vertices[mesh.faces[idx]]
        face_z = face_vertices[:, 2]
        # If the face is at min_z and normal is nearly (0,0,-1), it's the base, not an undercut
        if np.all(np.abs(face_z - min_z) < 0.01) and dots[idx] < -0.99:
            continue
        undercut_indices.append(idx)

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


def compute_part_hash(part: Part | Compound) -> str:
    """
    Computes a stable hash for a build123d Part or Compound.
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

def check_wall_thickness(
    mesh: trimesh.Trimesh, min_mm: float = 1.0, max_mm: float = float("inf")
) -> list[str]:
    """
    Functional check for wall thickness consistency using raycasting.
    """
    logger.debug("checking_wall_thickness", min_mm=min_mm, max_mm=max_mm)

    violations = []

    # Sample points on the mesh and cast rays along normals to find opposite wall
    # For MVP, we sample a subset of faces to keep it fast
    sample_size = min(len(mesh.faces), 1000)
    face_indices = np.random.choice(len(mesh.faces), sample_size, replace=False)

    centers = mesh.triangles_center[face_indices]
    # Inward normals
    normals = -mesh.face_normals[face_indices]

    # Offset origins slightly to avoid self-intersection
    origins = centers + normals * 1e-4

    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    locations, index_ray, _ = intersector.intersects_location(
        origins, normals, multiple_hits=False
    )

    if len(locations) > 0:
        # Distance between origin and hit point
        hit_origins = origins[index_ray]
        distances = np.linalg.norm(locations - hit_origins, axis=1)

        too_thin = np.where(distances < min_mm)[0]
        # Only check max thickness if it's finite
        too_thick = np.where(distances > max_mm)[0] if max_mm != float("inf") else []

        if len(too_thin) > 0:
            violations.append(
                f"Wall thickness too thin: {len(too_thin)} samples < {min_mm}mm"
            )
            logger.warning(
                "wall_too_thin", count=len(too_thin), min_dist=float(distances.min())
            )
        if len(too_thick) > 0:
            violations.append(
                f"Wall thickness too thick: {len(too_thick)} samples > {max_mm}mm"
            )
            logger.warning(
                "wall_too_thick", count=len(too_thick), max_dist=float(distances.max())
            )

    return violations
