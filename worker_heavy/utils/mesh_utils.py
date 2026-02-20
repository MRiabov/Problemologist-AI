import logging
import tempfile
from pathlib import Path
from typing import Literal

import trimesh

logger = logging.getLogger(__name__)


class MeshProcessingError(Exception):
    """Raised when mesh processing (repair or tetrahedralization) fails."""

    pass


def repair_mesh(mesh: trimesh.Trimesh, max_attempts: int = 3) -> trimesh.Trimesh:
    """Repairs a mesh to ensure it is watertight and manifold.

    Args:
        mesh: Input trimesh object.
        max_attempts: Number of times to try repairing the mesh.

    Returns:
        Repaired trimesh object.

    Raises:
        MeshProcessingError: If the mesh cannot be repaired after max_attempts.
    """
    for attempt in range(max_attempts):
        if mesh.is_watertight and mesh.is_winding_consistent:
            return mesh

        logger.info(
            f"Attempt {attempt + 1}/{max_attempts}: Mesh is not watertight or has inconsistent winding, attempting repair"
        )

        # process() merges vertices, removes duplicate/degenerate faces and unreferenced vertices
        mesh.process()

        # Repair normals and fill holes
        mesh.fix_normals()
        mesh.fill_holes()

    if not mesh.is_watertight:
        raise MeshProcessingError(
            "Mesh failed to become watertight after multiple repair attempts."
        )

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

    Raises:
        MeshProcessingError: If tetrahedralization fails.
    """
    if not input_path.exists():
        raise FileNotFoundError(f"Input mesh not found: {input_path}")

    try:
        if method == "gmsh":
            return _tetrahedralize_gmsh(input_path, output_msh_path, refine_level)
        if method == "tetgen":
            return _tetrahedralize_tetgen(input_path, output_msh_path)
        raise ValueError(f"Unknown tetrahedralization method: {method}")
    except Exception as e:
        logger.error(f"Tetrahedralization failed using {method}: {e}")
        raise MeshProcessingError(f"Tetrahedralization failed: {e}") from e


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
        gmsh.model.geo.addVolume([loop_tag])
        gmsh.model.geo.synchronize()

        # Optional: refine mesh size
        gmsh.option.setNumber("Mesh.MeshSizeFactor", refine_level)
        gmsh.option.setNumber("Mesh.Algorithm", 6)  # HXT for 3D
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 32)  # Refine near curves

        # Generate 3D mesh
        gmsh.model.mesh.generate(3)

        # Verify that 3D elements were actually created
        # getElements(3) returns (elementTypes, elementTags, nodeTags)
        elem_types, _, _ = gmsh.model.mesh.getElements(3)
        if len(elem_types) == 0:
            raise RuntimeError(
                "Gmsh failed to generate 3D tetrahedral elements. Check if the surface is closed and manifold."
            )

        # Optimize 3D mesh for better quality
        gmsh.model.mesh.optimize("Netgen")

        # Save as .msh
        output_msh_path.parent.mkdir(parents=True, exist_ok=True)
        gmsh.write(str(output_msh_path))

        return output_msh_path
    finally:
        if gmsh.isInitialized():
            gmsh.finalize()


def _tetrahedralize_tetgen(input_path: Path, output_msh_path: Path) -> Path:
    """Fallback implementation using TetGen CLI."""
    import subprocess

    import gmsh

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_stl = Path(tmpdir) / input_path.name
        tmp_stl.write_bytes(input_path.read_bytes())

        # -p: Tetrahedralize a piecewise linear complex
        # -q: Quality mesh generation. A minimum radius-edge ratio may be specified.
        cmd = ["tetgen", "-pq1.2", str(tmp_stl)]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            raise RuntimeError(f"TetGen failed: {result.stderr}")

        # Parse .node file
        node_file = tmp_stl.with_suffix(".1.node")
        if not node_file.exists():
            raise FileNotFoundError(f"TetGen output .node file not found: {node_file}")

        nodes = []
        node_tags = []
        with node_file.open() as f:
            # First line: <# nodes> <dim (3)> <# attributes> <# boundary markers>
            _header = f.readline()
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                # <index> <x> <y> <z> ...
                idx = int(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                nodes.extend([x, y, z])
                node_tags.append(idx)

        # Parse .ele file
        ele_file = tmp_stl.with_suffix(".1.ele")
        if not ele_file.exists():
            raise FileNotFoundError(f"TetGen output .ele file not found: {ele_file}")

        element_tags = []
        node_indices = []  # flat list of node tags for elements
        with ele_file.open() as f:
            # First line: <# tetrahedra> <nodes per tet (4)> <# attributes>
            _header = f.readline()
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                # <index> <n1> <n2> <n3> <n4> ...
                idx = int(parts[0])
                # Append 4 node tags
                node_indices.extend(
                    [int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])]
                )
                element_tags.append(idx)

        # Use Gmsh to write MSH file
        try:
            if not gmsh.isInitialized():
                gmsh.initialize()

            gmsh.model.add("TetGenModel")

            # Add a discrete volume entity (tag 1)
            vol_tag = 1
            gmsh.model.addDiscreteEntity(3, vol_tag)

            # Add nodes
            # addNodes(dim, entityTag, nodeTags, coord, parametricCoord=None)
            gmsh.model.mesh.addNodes(3, vol_tag, node_tags, nodes)

            # Add elements
            # addElements(dim, entityTag, elementTypes, elementTags, nodeTags)
            # Type 4 is 4-node tetrahedron
            gmsh.model.mesh.addElements(
                3, vol_tag, [4], [element_tags], [node_indices]
            )

            output_msh_path.parent.mkdir(parents=True, exist_ok=True)
            gmsh.write(str(output_msh_path))

            return output_msh_path

        finally:
            if gmsh.isInitialized():
                gmsh.finalize()
