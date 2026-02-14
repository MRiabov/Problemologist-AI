import numpy as np
import trimesh
import hashlib
from build123d import Compound, Part

def part_to_trimesh(part: Part | Compound) -> trimesh.Trimesh:
    """
    Converts a build123d Part or Compound to a trimesh.Trimesh object.
    Uses in-memory tessellation for performance.
    """
    # T016: Use direct tessellation to avoid disk I/O
    verts, faces = part.tessellate(tolerance=0.1, angular_tolerance=0.1)
    vertices = np.array([[v.X, v.Y, v.Z] for v in verts])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh

def compute_part_hash(part: Part | Compound) -> str:
    """
    Computes a stable hash for a build123d Part or Compound.
    Uses in-memory tessellation for performance.
    """
    # T016: Use direct tessellation to avoid disk I/O
    verts, faces = part.tessellate(tolerance=0.1, angular_tolerance=0.1)

    # Convert to stable bytes representation for hashing
    # We round to 4 decimal places to avoid tiny floating point variations
    vertices = np.array([[v.X, v.Y, v.Z] for v in verts], dtype=np.float32)
    vertices = np.round(vertices, 4)
    faces_arr = np.array(faces, dtype=np.int32)

    data = vertices.tobytes() + faces_arr.tobytes()
    return hashlib.sha256(data).hexdigest()
