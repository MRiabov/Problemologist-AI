import os
import io
import tempfile
import xml.etree.ElementTree as ET
from typing import List, Dict, Optional
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


class SceneCompiler:
    """
    Assembles MJCF (MuJoCo XML) from CAD geometry and agent specifications.
    """

    def __init__(self, asset_dir: Optional[str] = None):
        self.asset_dir = asset_dir
        if self.asset_dir and not os.path.exists(self.asset_dir):
            os.makedirs(self.asset_dir)

        self.root = ET.Element("mujoco", model="problemologist_scene")
        self._setup_basic_xml()

    def _setup_basic_xml(self):
        # Options
        ET.SubElement(self.root, "option", timestep="0.002", gravity="0 0 -9.81")

        # Visual settings
        visual = ET.SubElement(self.root, "visual")
        ET.SubElement(
            visual,
            "headlight",
            ambient=".4 .4 .4",
            diffuse=".8 .8 .8",
            specular="0.1 0.1 0.1",
        )
        ET.SubElement(visual, "map", znear="0.01")
        ET.SubElement(visual, "quality", shadowsize="2048")

        # Compiler
        ET.SubElement(self.root, "compiler", angle="degree", coordinate="local")
        if self.asset_dir:
            # We assume meshes are in the same dir as the XML or we use absolute paths
            # Actually, mujoco uses meshdir. But if we save them to self.asset_dir,
            # and the XML is also there, it works fine.
            pass

        # Assets (Placeholder for mesh assets)
        self.asset = ET.SubElement(self.root, "asset")
        # Texture and Material for floor
        ET.SubElement(
            self.asset,
            "texture",
            name="grid",
            type="2d",
            builtin="checker",
            rgb1=".1 .2 .3",
            rgb2=".2 .3 .4",
            width="300",
            height="300",
            mark="edge",
            markrgb=".8 .8 .8",
        )
        ET.SubElement(
            self.asset,
            "material",
            name="grid",
            texture="grid",
            texrepeat="1 1",
            texuniform="true",
        )

        # Worldbody
        self.worldbody = ET.SubElement(self.root, "worldbody")
        # Light
        ET.SubElement(
            self.worldbody,
            "light",
            pos="0 0 5",
            dir="0 0 -1",
            diffuse="1 1 1",
            specular=".3 .3 .3",
            directional="true",
        )
        # Floor
        ET.SubElement(
            self.worldbody,
            "geom",
            name="floor",
            type="plane",
            size="10 10 .1",
            material="grid",
            condim="3",
            conaffinity="15",
        )

    def compile(
        self,
        env_compound: Compound,
        agent_compound: Optional[Compound] = None,
        agent_joints: Optional[List[Dict]] = None,
    ) -> str:
        """
        Compiles the environment and agent into an MJCF XML string.
        """
        # Process Environment (T006)
        self._process_environment(env_compound)

        # Process Agent (T007)
        if agent_compound:
            self._process_agent(agent_compound, agent_joints)

        # Convert to string
        return ET.tostring(self.root, encoding="unicode", method="xml")

    def _process_environment(self, env_compound: Compound):
        """Processes the environment solids (zones and obstacles)."""
        for i, solid in enumerate(env_compound.solids()):
            # build123d objects might have a label attribute set by the user
            label = getattr(solid, "label", "")
            if not label:
                label = f"solid_{i}"

            if label.startswith("zone_"):
                self._add_zone_site(solid, label)
            elif label.startswith("obstacle_"):
                self._add_obstacle_geom(solid, label)
            else:
                # Default behavior for unnamed solids: treat as static obstacle
                self._add_obstacle_geom(solid, label)

    def _add_zone_site(self, solid: Solid, label: str):
        """Adds a non-colliding visual site for a zone."""
        center = solid.center()
        # For zones, we use a site with a specific color
        rgba = "0 1 0 0.3"  # Default green
        if "goal" in label:
            rgba = "0 1 0 0.5"  # Brighter green
        elif "forbid" in label:
            rgba = "1 0 0 0.5"  # Red
        elif "start" in label:
            rgba = "0 0 1 0.5"  # Blue

        # We use geom with conaffinity/contype 0 for better volume visualization
        ET.SubElement(
            self.worldbody,
            "geom",
            name=label,
            type="box",  # Approximate with box for now or use mesh if complex
            pos=f"{center.X} {center.Y} {center.Z}",
            size="0.05 0.05 0.05",  # Should ideally match solid bounds
            rgba=rgba,
            conaffinity="0",
            contype="0",
            group="1",  # Rendering group for visual-only
        )

        # Also add a site for detection point
        ET.SubElement(
            self.worldbody,
            "site",
            name=f"site_{label}",
            pos=f"{center.X} {center.Y} {center.Z}",
            size="0.01",
            rgba=rgba,
        )

    def _add_obstacle_geom(self, solid: Solid, label: str):
        """Adds a colliding mesh geom for an obstacle."""
        # T006: Call MeshProcessor -> Save mesh asset -> Add <geom type="mesh"/>
        stl_data = MeshProcessor.export_stl(solid)

        mesh_name = label
        mesh_filename = f"{label}.stl"

        if self.asset_dir:
            file_path = os.path.join(self.asset_dir, mesh_filename)
            with open(file_path, "wb") as f:
                f.write(stl_data)

        # Add mesh to asset
        ET.SubElement(self.asset, "mesh", name=mesh_name, file=mesh_filename)

        # Add geom to worldbody
        # Note: we use pos="0 0 0" because the mesh is already in global coordinates
        ET.SubElement(
            self.worldbody,
            "geom",
            name=f"geom_{label}",
            type="mesh",
            mesh=mesh_name,
            rgba="0.7 0.7 0.7 1",
        )

    def _process_agent(
        self, agent_compound: Compound, agent_joints: Optional[List[Dict]]
    ):
        """Injects agent bodies, joints, and actuators."""
        # T007: Implement Actuator Injection
        actuator_root = ET.SubElement(self.root, "actuator")

        # Create a body for the agent
        agent_name = "agent"
        agent_root = ET.SubElement(self.worldbody, "body", name=agent_name, pos="0 0 0")

        # Add agent geoms (simplified: treat whole compound as one body if no joints)
        # In a real scenario, we might want to split it.
        if not agent_joints:
            self._add_agent_meshes_to_body(agent_compound, agent_root, "agent_base")
        else:
            # If we have joints, we need to split the meshes.
            # For now, let's just add the base part (anything not in joints)
            self._add_agent_meshes_to_body(agent_compound, agent_root, "agent_base")

            for i, j_data in enumerate(agent_joints):
                j_name = j_data.get("name", f"joint_{i}")
                j_pos = j_data.get("pos", "0 0 0")
                j_axis = j_data.get("axis", "0 0 1")
                j_type = j_data.get("type", "hinge")

                # Child body
                child_body = ET.SubElement(
                    agent_root, "body", name=f"link_{i}", pos=j_pos
                )
                ET.SubElement(
                    child_body, "joint", name=j_name, type=j_type, axis=j_axis
                )

                # Add placeholder geom so the body has mass/inertia
                ET.SubElement(
                    child_body,
                    "geom",
                    name=f"geom_link_{i}",
                    type="sphere",
                    size="0.02",
                    rgba="0 0 1 1",
                )

                # Actuator
                ET.SubElement(
                    actuator_root,
                    "motor",
                    name=f"motor_{j_name}",
                    joint=j_name,
                    gear="10",
                )

    def _add_agent_meshes_to_body(
        self, compound: Compound, body_element: ET.Element, prefix: str
    ):
        """Helper to add meshes from a compound to a specific body element."""
        for i, solid in enumerate(compound.solids()):
            label = f"{prefix}_{i}"
            stl_data = MeshProcessor.export_stl(solid)

            mesh_filename = f"{label}.stl"
            if self.asset_dir:
                file_path = os.path.join(self.asset_dir, mesh_filename)
                with open(file_path, "wb") as f:
                    f.write(stl_data)

            ET.SubElement(self.asset, "mesh", name=label, file=mesh_filename)
            ET.SubElement(body_element, "geom", type="mesh", mesh=label, rgba="0 0 1 1")

