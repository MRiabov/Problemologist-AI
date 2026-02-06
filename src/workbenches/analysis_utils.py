import hashlib
import yaml
import numpy as np
import trimesh
from build123d import Part

def load_config() -> dict:
    with open("config/manufacturing_config.yaml", "r") as f:
        return yaml.safe_load(f)

def part_to_trimesh(part: Part) -> trimesh.Trimesh:
    """
    Converts a build123d Part to a trimesh.Trimesh object.
    """
    # tessellate returns (vertices, triangles)
    # vertices is a list of Vector objects
    # triangles is a list of 3-tuples of indices
    verts_objs, faces = part.tessellate(1e-3)

    # Convert Vector objects to [x, y, z] lists
    vertices = [[v.X, v.Y, v.Z] for v in verts_objs]

    return trimesh.Trimesh(vertices=vertices, faces=faces)

def compute_part_hash(part: Part) -> str:
    """
    Computes a stable hash for the part based on geometry.
    """
    # Using volume and center of mass as a proxy for geometry
    c = part.center()
    data = f"{part.volume:.6f}_{c.X:.6f}_{c.Y:.6f}_{c.Z:.6f}"
    return hashlib.md5(data.encode()).hexdigest()

def check_undercuts(mesh: trimesh.Trimesh, direction_vector: tuple[float, float, float]) -> list[int]:
    """
    Checks for undercuts in the given pull direction.
    Returns a list of face indices that are undercut (occluded).
    """
    # 1. Normalize direction
    direction = np.array(direction_vector)
    direction = direction / np.linalg.norm(direction)

    # 2. Raycast from face centers in the direction of pull
    origins = mesh.triangles_center.copy()

    # Offset slightly along normal to avoid self-intersection at origin
    origins += mesh.face_normals * 1e-5

    # Create rays: one per face, pointing in pull direction
    # We want to know if these rays hit anything.
    # If they hit something, the face is occluded from the pull direction (undercut).

    # Prepare ray directions
    vectors = np.tile(direction, (len(origins), 1))

    # Use trimesh's ray intersector
    intersects = mesh.ray.intersects_any(origins, vectors)

    # Return indices of faces that intersected something
    return np.where(intersects)[0].tolist()

def check_draft_angle(mesh: trimesh.Trimesh, pull_vector: tuple[float, float, float], min_angle_deg: float) -> list[int]:
    """
    Checks if faces meet the minimum draft angle requirement.
    Returns list of face indices that violate the draft angle.
    """
    # Normalize pull vector
    pv = np.array(pull_vector)
    pv = pv / np.linalg.norm(pv)

    # Calculate angle between face normals and pull vector
    # faces are valid if angle is NOT close to 90 degrees.
    # Specifically, we want the angle from the "vertical" (relative to pull) to be >= min_angle.
    # Vertical means normal is perpendicular to pull. Dot product is 0.
    # Angle from vertical = abs(90 - angle_from_pull).

    normals = mesh.face_normals
    dots = np.dot(normals, pv)
    # Clip for safety
    dots = np.clip(dots, -1.0, 1.0)

    angles_rad = np.arccos(dots)
    angles_deg = np.degrees(angles_rad)

    # Draft angle is the deviation from perpendicular (90 deg)
    # e.g. if normal is 90 deg to pull, draft is 0.
    # if normal is 88 deg, draft is 2 deg.
    current_draft = np.abs(90.0 - angles_deg)

    # Violations are where draft < min_angle
    violations = np.where(current_draft < min_angle_deg)[0]

    return violations.tolist()

def analyze_wall_thickness(mesh: trimesh.Trimesh) -> dict[str, float]:
    """
    Analyzes wall thickness statistics.
    Placeholder implementation.
    """
    # Real implementation requires complex raycasting or SDF.
    return {
        "min_mm": 1.0,
        "max_mm": 5.0,
        "average_mm": 2.5
    }

def check_wall_thickness(mesh: trimesh.Trimesh, min_mm: float, max_mm: float) -> list[int]:
    """
    Checks for wall thickness violations.
    Placeholder implementation.
    """
    return []
