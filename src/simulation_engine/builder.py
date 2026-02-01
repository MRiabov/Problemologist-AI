import io
import tempfile
import trimesh
from build123d import Solid, Compound, export_stl


class MeshProcessor:
    """
    Handles conversion between build123d geometry and physics-ready meshes.
    """

    @staticmethod
    def export_stl(shape: Solid | Compound) -> bytes:
        """
        Exports a build123d shape to STL format as bytes.
        """
        with tempfile.NamedTemporaryFile(suffix=".stl", delete=True) as tmp:
            export_stl(shape, tmp.name)
            tmp.seek(0)
            return tmp.read()

    @staticmethod
    def load_mesh(stl_data: bytes) -> trimesh.Trimesh:
        """
        Loads STL data into a trimesh.Trimesh object.
        """
        file_obj = io.BytesIO(stl_data)
        mesh = trimesh.load(file_obj, file_type="stl")

        # trimesh.load can return a Scene if multiple meshes are present.
        # MuJoCo geoms usually expect a single mesh.
        if isinstance(mesh, trimesh.Scene):
            if len(mesh.geometry) == 0:
                raise ValueError("No geometry found in STL data.")
            # Concatenate all geometries in the scene into one mesh
            mesh = trimesh.util.concatenate(list(mesh.geometry.values()))

        return mesh

    @staticmethod
    def compute_convex_hull(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """
        Computes the convex hull of the given mesh.
        Returns a new trimesh.Trimesh object representing the convex hull.
        """
        hull = mesh.convex_hull
        return hull
