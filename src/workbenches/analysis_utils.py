import hashlib
import tempfile
from pathlib import Path
from typing import Any

import numpy as np
import trimesh
import yaml
from build123d import Part, export_stl


def load_config() -> dict[str, Any]:
    """
    Loads the manufacturing configuration from config/manufacturing_config.yaml.
    """
    # Find the config file relative to the project root
    current_dir = Path(__file__).resolve().parent
    # Traversing up to find the root. Assuming src/workbenches/analysis_utils.py
    project_root = current_dir.parent.parent
    config_path = project_root / "config" / "manufacturing_config.yaml"

    if not config_path.exists():
        # Fallback: try to find 'config' directory by walking up
        search_dir = current_dir
        while search_dir.parent != search_dir:
            if (search_dir / "config" / "manufacturing_config.yaml").exists():
                config_path = search_dir / "config" / "manufacturing_config.yaml"
                break
            search_dir = search_dir.parent

    if not config_path.exists():
         raise FileNotFoundError("Could not find config/manufacturing_config.yaml")

    with config_path.open("r") as f:
        return yaml.safe_load(f)

def part_to_trimesh(part: Part) -> trimesh.Trimesh:
    """
    Converts a build123d Part to a trimesh.Trimesh object.
    """
    # Export to STL in memory
    # Using a named temporary file to ensure trimesh can read it
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        export_stl(part, tmp.name)
        tmp_path = tmp.name

    try:
        mesh = trimesh.load(tmp_path)
    finally:
        p = Path(tmp_path)
        if p.exists():
            p.unlink()

    return mesh

def compute_part_hash(part: Part) -> str:
    """
    Computes a hash for the part based on its volume and center of mass.
    This is a heuristic for part equality.
    """
    # Using volume and center as a proxy for geometry hash
    center = part.center()
    data = f"{part.volume:.6f}-{center.X:.4f},{center.Y:.4f},{center.Z:.4f}"
    return hashlib.md5(data.encode()).hexdigest()

def check_undercuts(mesh: trimesh.Trimesh, direction: tuple[float, float, float]) -> list[int]:
    """
    Checks for undercuts in the given direction.
    Returns a list of face indices that are undercuts.

    An undercut is a face that is not visible from the given direction (infinity).
    We assume the tool/mold approaches from 'direction'.
    So we check if the face is visible from 'infinity * direction'.
    This is equivalent to casting a ray from the face center in the 'direction'.
    If it hits the mesh, it is occluded.
    """
    # Normalize direction
    direction = np.array(direction)
    norm = np.linalg.norm(direction)
    if norm > 0:
        direction = direction / norm

    # Ray origins: Face centers + epsilon * direction
    # We add a small offset to avoid self-intersection with the face itself
    ray_origins = mesh.triangles_center + (mesh.face_normals * 1e-5)

    # Ray directions: direction
    ray_directions = np.tile(direction, (len(mesh.faces), 1))

    # Run ray query
    # Using the high-level API which handles intersector selection
    _ = mesh.ray.intersects_id(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False
    )

    # index_tri contains indices of faces that were hit by rays.
    # The rays originate from *every* face.
    # The return value of intersects_id is an array of face indices that were hit.
    # But wait, intersects_id returns face indices of the *mesh* that were hit.
    # We need to know *which rays* hit something.

    # Let's use intersects_id with return_locations=False, but it returns face indices hit.
    # This doesn't tell us which ray hit.

    # We must use RayMeshIntersector directly to get ray indices.
    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    _, index_ray, _ = intersector.intersects_id(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False,
        return_locations=True
    )

    # index_ray contains indices of rays that hit something.
    # Since ray i corresponds to face i, these are the faces that are occluded.

    return list(set(index_ray))

def check_draft_angle(
    mesh: trimesh.Trimesh,
    pull_vector: tuple[float, float, float],
    min_angle_deg: float,
) -> list[int]:
    """
    Checks if faces have sufficient draft angle relative to the pull vector.
    Returns list of face indices that violate the draft angle.
    """
    pull_vector = np.array(pull_vector)
    norm = np.linalg.norm(pull_vector)
    if norm > 0:
        pull_vector = pull_vector / norm

    normals = mesh.face_normals
    dots = np.dot(normals, pull_vector)

    # Draft angle is angle between surface tangent and pull vector.
    # Which is (90 - angle(normal, pull_vector)).
    # We want draft >= min_angle.
    # So 90 - angle(normal, pull) >= min_angle
    # angle(normal, pull) <= 90 - min_angle.

    # AND for the other side (if normal points away):
    # angle(normal, pull) >= 90 + min_angle.

    # So we want angle(normal, pull) to be outside the range (90-min, 90+min).
    # Cosine of angle should be outside (-sin(min), sin(min)).
    # i.e., abs(dot) >= sin(min_angle).

    # However, faces perpendicular to pull vector (top/bottom) have dot ~ 1 or -1. These are fine (draft = 90).
    # Faces parallel to pull vector (vertical walls) have dot ~ 0. These are the problem.
    # If dot is 0, angle is 90, draft is 0.

    min_sin = np.sin(np.radians(min_angle_deg))

    violations = []
    for i, dot in enumerate(dots):
        if abs(dot) < min_sin:
            violations.append(i)

    return violations

def check_wall_thickness(
    mesh: trimesh.Trimesh, min_mm: float, max_mm: float
) -> list[int]:
    """
    Checks for wall thickness violations.
    For each face, cast a ray inwards (opposite to normal).
    Distance to next intersection is thickness.
    """
    ray_origins = mesh.triangles_center - (mesh.face_normals * 1e-5)
    ray_directions = -mesh.face_normals  # Inwards

    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    _, index_ray, locations = intersector.intersects_id(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False,
        return_locations=True,
    )

    violations = []

    # Map back to faces
    for i, ray_idx in enumerate(index_ray):
        origin = ray_origins[ray_idx]
        hit_loc = locations[i]
        dist = np.linalg.norm(hit_loc - origin)

        if dist < min_mm or dist > max_mm:
            violations.append(ray_idx)

    return violations

def analyze_wall_thickness(mesh: trimesh.Trimesh) -> dict[str, float]:
    """
    Returns statistics about wall thickness.
    """
    ray_origins = mesh.triangles_center - (mesh.face_normals * 1e-5)
    ray_directions = -mesh.face_normals  # Inwards

    intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    _, index_ray, locations = intersector.intersects_id(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
        multiple_hits=False,
        return_locations=True,
    )

    distances = []
    for i, ray_idx in enumerate(index_ray):
        origin = ray_origins[ray_idx]
        hit_loc = locations[i]
        dist = np.linalg.norm(hit_loc - origin)
        distances.append(dist)

    if not distances:
        return {"min_mm": 0.0, "max_mm": 0.0, "average_mm": 0.0}

    return {
        "min_mm": float(np.min(distances)),
        "max_mm": float(np.max(distances)),
        "average_mm": float(np.mean(distances))
    }
