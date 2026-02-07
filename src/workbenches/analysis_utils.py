import hashlib
import tempfile
import pathlib
import trimesh
import numpy as np
from build123d import Part, Compound
from typing import Union, List, Tuple

def part_to_trimesh(part: Union[Part, Compound]) -> trimesh.Trimesh:
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

def check_undercuts(mesh: trimesh.Trimesh, approach_direction: Tuple[float, float, float] = (0, 0, 1)) -> List[int]:
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

def compute_part_hash(part: Union[Part, Compound]) -> str:
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
