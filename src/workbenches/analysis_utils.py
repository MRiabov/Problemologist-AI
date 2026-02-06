import os
import yaml
import trimesh
import hashlib
import tempfile
import numpy as np
from build123d import Part, export_stl

def load_config():
    """
    Loads the manufacturing configuration from config/manufacturing_config.yaml.
    """
    config_path = os.path.join("config", "manufacturing_config.yaml")
    if not os.path.exists(config_path):
        # Fallback for different CWD (e.g. inside tests)
        # Try to find the config directory by walking up
        curr = os.path.abspath(os.path.dirname(__file__))
        prev = None
        while curr != prev:
            check_path = os.path.join(curr, "config", "manufacturing_config.yaml")
            if os.path.exists(check_path):
                config_path = check_path
                break
            prev = curr
            curr = os.path.dirname(curr)

    with open(config_path, "r") as f:
        return yaml.safe_load(f)

def part_to_trimesh(part: Part) -> trimesh.Trimesh:
    """
    Converts a build123d Part to a trimesh.Trimesh object.
    """
    # Create temp file, close it immediately so export_stl can write to it safely
    # This prevents file locking issues on Windows
    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        export_stl(part, tmp_path)
        mesh = trimesh.load(tmp_path)
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)

    return mesh

def compute_part_hash(part: Part) -> str:
    """
    Computes a hash for the part based on its geometric properties.
    """
    # Simple hash based on volume, center of mass, and area
    # This is not perfect but sufficient for caching context in workbenches
    data = f"{part.volume:.4f}_{part.area:.4f}_{part.center()}"
    return hashlib.md5(data.encode()).hexdigest()

def analyze_wall_thickness(mesh: trimesh.Trimesh, sample_count: int = 1000) -> dict:
    """
    Analyzes wall thickness by raycasting from surface points inwards.
    Returns min, max, average thickness.
    """
    if len(mesh.faces) == 0:
         return {"min_mm": 0.0, "max_mm": 0.0, "average_mm": 0.0}

    # Sample points on the surface
    points, face_indices = trimesh.sample.sample_surface(mesh, sample_count)
    normals = mesh.face_normals[face_indices]

    # Raycast inwards (opposite to normal)
    # Offset origin slightly to avoid self-intersection at start
    origins = points - (normals * 1e-3)
    vectors = -normals

    # Use raycaster
    # result is locations, index_ray, index_tri
    # we want the first hit for each ray
    intersections = mesh.ray.intersects_location(
        ray_origins=origins,
        ray_directions=vectors,
        multiple_hits=False
    )

    # intersects_location returns (points, ray_indices, face_indices)
    hit_points = intersections[0]
    ray_indices = intersections[1]

    if len(ray_indices) == 0:
        # Fallback if somehow no rays hit (e.g. single surface?)
        return {"min_mm": 0.0, "max_mm": 0.0, "average_mm": 0.0}

    # Calculate distances
    # We only care about rays that hit something.
    # ray_indices tells us which original point corresponds to the hit.

    original_points = points[ray_indices]
    distances = np.linalg.norm(hit_points - original_points, axis=1)

    if len(distances) == 0:
         return {"min_mm": 0.0, "max_mm": 0.0, "average_mm": 0.0}

    return {
        "min_mm": float(np.min(distances)),
        "max_mm": float(np.max(distances)),
        "average_mm": float(np.mean(distances))
    }

def check_draft_angle(mesh: trimesh.Trimesh, pull_vector: tuple, min_angle_deg: float) -> list:
    """
    Checks for draft angle violations.
    Returns a list of face indices that violate the draft angle.
    A face violates draft if its normal is within (90 +/- min_angle) degrees of the pull vector.
    """
    pull_vec = np.array(pull_vector)
    pull_vec = pull_vec / np.linalg.norm(pull_vec)

    # Dot product of face normals with pull vector
    # dot = cos(theta)
    dots = np.dot(mesh.face_normals, pull_vec)

    # Clip to avoid numerical errors
    dots = np.clip(dots, -1.0, 1.0)

    # Angle in radians
    angles_rad = np.arccos(dots)
    angles_deg = np.degrees(angles_rad)

    # We want faces to be either roughly parallel to pull (0 deg) or roughly anti-parallel (180 deg) is NOT draft.
    # Draft is about ease of ejection. The walls (which are roughly 90 deg to pull) need to be slanted.
    # So if angle is 90, it's 0 draft.
    # Valid ranges: [0, 90 - min] or [90 + min, 180].
    # Violation range: (90 - min) < angle < (90 + min)

    lower_bound = 90.0 - min_angle_deg
    upper_bound = 90.0 + min_angle_deg

    violations = np.where((angles_deg > lower_bound) & (angles_deg < upper_bound))[0]

    return violations.tolist()

def check_undercuts(mesh: trimesh.Trimesh, direction: tuple) -> list:
    """
    Checks for undercuts by raycasting from face centers in the pull direction.
    Returns a list of face indices that are occluded (undercuts).
    """
    direction_vec = np.array(direction)
    direction_vec = direction_vec / np.linalg.norm(direction_vec)

    # Origins: Face centers, offset slightly along face normal to avoid self-intersection/grazing
    # As per memory/best practice, offset along normal prevents ray traveling inside the face for vertical walls
    origins = mesh.triangles_center + (mesh.face_normals * 1e-5)

    # Directions: All same direction
    vectors = np.tile(direction_vec, (len(mesh.faces), 1))

    # Raycast
    is_occluded = mesh.ray.intersects_any(
        ray_origins=origins,
        ray_directions=vectors
    )

    # Get indices where is_occluded is True
    undercut_faces = np.where(is_occluded)[0]

    return undercut_faces.tolist()

def check_wall_thickness(mesh: trimesh.Trimesh, min_mm: float, max_mm: float) -> list:
    """
    Checks wall thickness constraints.
    Returns list of face indices (approximated) that violate thickness.
    Note: Since thickness is volume-based, returning faces is a heuristic.
    We return indices of faces whose corresponding sample points violated thickness.
    """
    # Use logic similar to analyze_wall_thickness but keep track of violations

    # We'll verify thickness at face centers for efficiency
    origins = mesh.triangles_center
    normals = mesh.face_normals

    # Shoot ray inward
    ray_origins = origins - (normals * 1e-3)
    ray_vectors = -normals

    intersections = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_vectors,
        multiple_hits=False
    )

    hit_points = intersections[0]
    ray_indices = intersections[1]

    if len(ray_indices) == 0:
        return []

    # Check distances
    distances = np.linalg.norm(hit_points - ray_origins[ray_indices], axis=1)

    # Check violations
    # ray_indices maps back to face indices (0 to N-1) because we used all faces

    violation_indices = []

    for i, dist in zip(ray_indices, distances):
        if dist < min_mm or dist > max_mm:
            violation_indices.append(int(i))

    return violation_indices
