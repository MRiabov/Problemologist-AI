import io
import functools
from pathlib import Path
import yaml
import trimesh
import numpy as np
from build123d import Part, export_stl
import tempfile
import os

@functools.lru_cache(maxsize=1)
def load_config() -> dict:
    """Loads the manufacturing configuration from the local YAML file."""
    config_path = Path(__file__).parent / "manufacturing_config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(f"Manufacturing config not found at {config_path}")
    
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def part_to_trimesh(part: Part) -> trimesh.Trimesh:
    """Converts a build123d Part to a trimesh Trimesh object via STL."""
    # Use a temporary file as some build123d versions might have issues with BytesIO
    fd, path = tempfile.mkstemp(suffix=".stl")
    try:
        os.close(fd)
        export_stl(part, path)
        if os.path.getsize(path) == 0:
            raise ValueError("export_stl produced an empty file.")
        mesh = trimesh.load(path, file_type='stl')
    finally:
        if os.path.exists(path):
            os.remove(path)
    
    if isinstance(mesh, trimesh.Scene):
        if len(mesh.geometry) == 0:
            raise ValueError("No geometry found in the converted STL.")
        # Concatenate all geometries in the scene into a single mesh
        # We need to apply the scene graph transforms
        meshes = []
        for node_name in mesh.graph.nodes_geometry:
            transform, geometry_name = mesh.graph[node_name]
            geometry = mesh.geometry[geometry_name].copy()
            geometry.apply_transform(transform)
            meshes.append(geometry)
        mesh = trimesh.util.concatenate(meshes)
        
    return mesh

def check_draft_angle(mesh: trimesh.Trimesh, pull_vector: tuple, min_angle_deg: float) -> list[int]:
    """
    Identifies faces that are too steep relative to the pull direction.
    Returns a list of face indices where the draft angle is insufficient.
    """
    pull_vec = np.array(pull_vector, dtype=float)
    pull_vec = pull_vec / np.linalg.norm(pull_vec)
    
    # Get normals
    normals = mesh.face_normals
    
    # Calculate angle between normal and pull vector
    dot_products = np.dot(normals, pull_vec)
    dot_products = np.clip(dot_products, -1.0, 1.0)
    angles_rad = np.arccos(dot_products)
    angles_deg = np.degrees(angles_rad)
    
    # Insufficient draft if angle to pull vector is too close to 90 degrees
    insufficient_draft_indices = np.where(np.abs(90 - angles_deg) < min_angle_deg)[0]
    
    return insufficient_draft_indices.tolist()

def check_undercuts(mesh: trimesh.Trimesh, pull_vector: tuple) -> list[int]:
    """
    Detects faces occluded from the pull direction using raycasting.
    Only faces pointing away from the pull direction are candidates for undercuts.
    Returns a list of face indices that are occluded (undercuts).
    """
    pull_vec = np.array(pull_vector, dtype=float)
    pull_vec = pull_vec / np.linalg.norm(pull_vec)
    
    normals = mesh.face_normals
    dots = np.dot(normals, pull_vec)
    candidate_indices = np.where(dots < -0.1)[0]
    
    if len(candidate_indices) == 0:
        return []

    epsilon = 1e-4
    origins = mesh.triangles_center[candidate_indices] + (pull_vec * epsilon)
    directions = np.tile(pull_vec, (len(origins), 1))
    
    occluded = mesh.ray.intersects_any(origins, directions)
    actual_undercuts = candidate_indices[occluded]
    return actual_undercuts.tolist()

def check_wall_thickness(mesh: trimesh.Trimesh, min_mm: float, max_mm: float) -> list[int]:
    """
    Approximates wall thickness by casting rays inwards from face centers.
    Returns a list of face indices with thickness outside [min_mm, max_mm].
    """
    normals = mesh.face_normals
    epsilon = 1e-4
    origins = mesh.triangles_center - (normals * epsilon)
    directions = -normals
    
    locations, index_ray, index_tri = mesh.ray.intersects_location(origins, directions, multiple_hits=False)
    
    distances = np.full(len(mesh.faces), np.inf)
    if len(index_ray) > 0:
        actual_origins = origins[index_ray]
        dist = np.linalg.norm(locations - actual_origins, axis=1)
        distances[index_ray] = dist
    
    violated_indices = np.where((distances < min_mm) | (distances > max_mm))[0]
    return violated_indices.tolist()