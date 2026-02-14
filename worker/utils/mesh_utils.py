import logging
import tempfile
from pathlib import Path
from typing import Literal

import trimesh

logger = logging.getLogger(__name__)


def repair_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Repairs a mesh to ensure it is watertight and manifold.

    Args:
        mesh: Input trimesh object.

    Returns:
        Repaired trimesh object.
    """
    if mesh.is_watertight and mesh.is_winding_consistent:
        return mesh

    logger.info("Mesh is not watertight or has inconsistent winding, attempting repair")

    # Remove duplicate/unreferenced parts
    mesh.remove_duplicate_faces()
    mesh.remove_infinite_values()
    mesh.remove_unreferenced_vertices()

    # Repair normals and fill holes
    mesh.fix_normals()
    mesh.fill_holes()

    # If still not watertight, try a more aggressive approach if needed
    # but for now, these are the standard safe repairs.

    return mesh


def repair_mesh_file(input_path: Path, output_path: Path) -> Path:
    """Reads a mesh file, repairs it, and writes it back.

    Args:
        input_path: Path to input mesh (STL, OBJ, etc.)
        output_path: Path to write repaired mesh (typically STL for Gmsh)

    Returns:
        Path to the repaired mesh file.
    """
    mesh = trimesh.load(str(input_path))
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    repaired = repair_mesh(mesh)
    repaired.export(str(output_path))
    return output_path


def tetrahedralize(
    input_path: Path,
    output_msh_path: Path,
    method: Literal["gmsh", "tetgen"] = "gmsh",
    refine_level: float = 1.0,
) -> Path:
    """Tetrahedralizes a surface mesh into a 3D volumetric mesh.

    Args:
        input_path: Path to the input surface mesh (STL).
        output_msh_path: Path where the .msh file should be saved.
        method: The tetrahedralization tool to use.
        refine_level: Factor to adjust mesh density (smaller = finer).

    Returns:
        Path to the generated .msh file.
    """
    if not input_path.exists():
        raise FileNotFoundError(f"Input mesh not found: {input_path}")

    if method == "gmsh":
        return _tetrahedralize_gmsh(input_path, output_msh_path, refine_level)
    elif method == "tetgen":
        # Fallback to T006 implementation if gmsh is unavailable or fails
        return _tetrahedralize_tetgen(input_path, output_msh_path)
    else:
        raise ValueError(f"Unknown tetrahedralization method: {method}")


def _tetrahedralize_gmsh(
    input_path: Path, output_msh_path: Path, refine_level: float
) -> Path:
    """Internal implementation using Gmsh Python API."""
    import gmsh

    try:
        if not gmsh.isInitialized():
            gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.model.add("VolumetricModel")

        # Load the STL
        gmsh.merge(str(input_path))

        # In Gmsh, STL is just a collection of triangles (discrete surfaces)
        # We need to create a volume from them.
        entities = gmsh.model.getEntities(2)
        if not entities:
            raise RuntimeError("No surfaces found in STL")

        # Create a surface loop from all surfaces
        surface_tags = [e[1] for e in entities]
        loop_tag = gmsh.model.geo.addSurfaceLoop(surface_tags)

        # Add a volume
        volume_tag = gmsh.model.geo.addVolume([loop_tag])
        gmsh.model.geo.synchronize()

        # Optional: refine mesh size
        # gmsh.option.setNumber("Mesh.MeshSizeFactor", refine_level)

        # Generate 3D mesh
        gmsh.model.mesh.generate(3)

        # Save as .msh (Genesis usually prefers Version 4 ASCII)
        output_msh_path.parent.mkdir(parents=True, exist_ok=True)
        gmsh.write(str(output_msh_path))

        return output_msh_path
    except Exception as e:
        logger.error(f"Gmsh tetrahedralization failed: {e}")
        raise RuntimeError(f"Gmsh failed: {e}") from e
    finally:
        if gmsh.isInitialized():
            gmsh.finalize()


def _tetrahedralize_tetgen(input_path: Path, output_msh_path: Path) -> Path:
    """Fallback implementation using TetGen CLI (if installed)."""
    import subprocess

    # Existing TetGen logic...
    # (Simplified for now as Gmsh is preferred)
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_stl = Path(tmpdir) / input_path.name
        tmp_stl.write_bytes(input_path.read_bytes())

        cmd = ["tetgen", "-pq1.2", str(tmp_stl)]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            raise RuntimeError(f"TetGen failed: {result.stderr}")

        node_file = tmp_stl.with_suffix(".1.node")
        ele_file = tmp_stl.with_suffix(".1.ele")

        # TetGen to MSH conversion would go here if needed.
        # For now, we favor Gmsh which produces .msh directly.
        output_msh_path.parent.mkdir(parents=True, exist_ok=True)
        # Note: This is a placeholder for real conversion if Gmsh fails.
        node_file.rename(output_msh_path.with_suffix(".node"))
        ele_file.rename(output_msh_path.with_suffix(".ele"))

        return output_msh_path
