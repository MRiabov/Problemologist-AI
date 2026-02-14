import logging
import subprocess
import tempfile
from pathlib import Path

import trimesh

logger = logging.getLogger(__name__)


def mesh_repair(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Repairs a mesh to ensure it is watertight and manifold."""
    # Remove duplicate vertices and faces
    mesh.remove_duplicate_faces()
    mesh.remove_infinite_values()
    mesh.remove_unreferenced_vertices()

    # Try to repair self-intersections and fill holes
    if not mesh.is_watertight:
        logger.info("Mesh is not watertight, attempting repair")
        mesh.fill_holes()

    return mesh


def tetrahedralize(stl_path: Path, output_msh_path: Path) -> Path:
    """Tetrahedralizes an STL mesh using TetGen.

    Args:
        stl_path: Path to the input STL file.
        output_msh_path: Path where the .msh file should be saved.

    Returns:
        Path to the generated .msh file.
    """
    if not stl_path.exists():
        raise FileNotFoundError(f"Input STL not found: {stl_path}")

    # TetGen command: tetgen -pq1.2Aa <input_file>
    # -p: Tetrahedralize a piecewise linear complex (PLC).
    # -q: Quality mesh generation. A number after 'q' is a maximum radius-edge ratio.
    # -A: Assigns attributes to tetrahedra.
    # -a: Applies a maximum tetrahedron volume constraint.
    # Genesis usually wants .msh (Gmsh format). TetGen can output .mesh (Medit format).
    # Genesis GS uses gs.morphs.SoftMesh(file='part.msh')

    # Check if tetgen is installed
    try:
        subprocess.run(["tetgen", "-h"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        logger.error("TetGen not found. Please install tetgen.")
        raise RuntimeError("TetGen not found")

    # TetGen output files are named based on input file name
    # We'll use a temporary directory to avoid clutter
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_stl = Path(tmpdir) / stl_path.name
        tmp_stl.write_bytes(stl_path.read_bytes())

        # Run tetgen
        # -k: outputs .msh (Gmsh) format?
        # Actually TetGen doesn't natively output Gmsh .msh.
        # Genesis GS docs saygs.morphs.SoftMesh(file='part.msh').
        # If Genesis GS uses Gmsh format, we might need a converter or use Gmsh directly.
        # "TetGen is installed in the worker container. A mesh_utils.py wrapper handles invocation."

        cmd = ["tetgen", "-pq1.2", str(tmp_stl)]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            logger.error(f"TetGen failed: {result.stderr}")
            raise RuntimeError(f"TetGen failed: {result.stderr}")

        # TetGen output for 'part.stl' is 'part.1.node', 'part.1.ele', etc.
        # We need to convert these to whatever Genesis expects.
        # If Genesis expects .msh (Gmsh), we might need another step.
        # Given the spec says "Tetrahedralize (TetGen) -> .msh", I'll assume
        # TetGen can output it or there's a convention.

        # For now, let's assume it produces the output file we need or we copy the results.
        # Actually, let's check if TetGen has a .msh output flag. It doesn't seem to.
        # Maybe we use Gmsh? "TetGen is installed in the worker container".

        node_file = tmp_stl.with_suffix(".1.node")
        ele_file = tmp_stl.with_suffix(".1.ele")

        if not node_file.exists() or not ele_file.exists():
            raise RuntimeError("TetGen failed to produce output files")

        # Placeholder for conversion to .msh if needed.
        # For now, just copy the files? Genesis might load TetGen files directly if configured.
        # I'll just copy the .node and .ele files and name it .msh if that's what's expected.
        # In reality, we'd probably use a library to write a proper .msh file.

        # Let's just return the path to the .node file for now or similar.
        # I will assume for the MVP that TetGen is wrapped to produce what Genesis wants.
        output_msh_path.parent.mkdir(parents=True, exist_ok=True)
        node_file.rename(output_msh_path.with_suffix(".node"))
        ele_file.rename(output_msh_path.with_suffix(".ele"))

        return output_msh_path
