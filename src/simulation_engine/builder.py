import io
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh
from build123d import Compound, Solid, export_stl

from src.cots.utils import get_description


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

    def __init__(self, asset_dir: str | None = None):
        self.asset_dir = asset_dir
        if self.asset_dir:
            asset_path = Path(self.asset_dir)
            if not asset_path.exists():
                asset_path.mkdir(parents=True, exist_ok=True)

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
        compiler = ET.SubElement(
            self.root, "compiler", angle="degree", coordinate="local"
        )
        if self.asset_dir:
            # If asset_dir is already relative, use it as is
            if not Path(self.asset_dir).is_absolute():
                compiler.set("meshdir", self.asset_dir)
            else:
                # Set meshdir. If it is inside /workspace/ (sandbox), use a relative path
                # to ensure MuJoCo inside the container can find it regardless of host mapping.
                path = Path(self.asset_dir).resolve()
                if str(path).startswith("/workspace/"):
                    # Make it relative to /workspace/
                    rel_path = path.relative_to("/workspace/")
                    compiler.set("meshdir", str(rel_path))
                else:
                    compiler.set("meshdir", str(path))

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
        agent_compound: Compound | None = None,
        agent_joints: list[dict] | None = None,
        env_labels: list[str] | None = None,
        agent_labels: list[str] | None = None,
    ) -> str:
        """
        Compiles the environment and agent into an MJCF XML string.
        """
        # Process Environment (T006)
        self._process_environment(env_compound, env_labels)

        # Process Agent (T007)
        if agent_compound:
            self._process_agent(agent_compound, agent_joints, agent_labels)

        # Convert to string
        return ET.tostring(self.root, encoding="unicode", method="xml")

    def _process_environment(
        self, env_compound: Compound, labels: list[str] | None = None
    ):
        """Processes the environment solids (zones and obstacles)."""
        for i, solid in enumerate(env_compound.solids()):
            # Use explicit label if provided, otherwise check object attribute
            label = None
            if labels and i < len(labels):
                label = labels[i]

            if not label:
                label = getattr(solid, "label", None)

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
            asset_path = Path(self.asset_dir)
            file_path = asset_path / mesh_filename
            file_path.write_bytes(stl_data)

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
        self,
        agent_compound: Compound,
        agent_joints: list[dict] | None,
        labels: list[str] | None = None,
    ):
        """Injects agent bodies, joints, and actuators."""
        # T007: Implement Actuator Injection
        actuator_root = ET.SubElement(self.root, "actuator")

        # Create a body for the agent
        agent_name = "agent"
        agent_root = ET.SubElement(self.worldbody, "body", name=agent_name, pos="0 0 0")

        # Identify parts
        parts = []
        if isinstance(agent_compound, Compound):
            parts = list(agent_compound)
        else:
            parts = [agent_compound]

        # Add agent geoms (simplified: treat whole compound as one body if no joints)
        if not agent_joints:
            base_label = labels[0] if labels else "agent_base"
            self._add_agent_meshes_to_body(agent_compound, agent_root, base_label)
        else:
            # If we have joints, we distribute parts:
            # Part 0 -> Base
            # Part i+1 -> Link i

            # Base part
            if parts:
                base_label = labels[0] if labels else "agent_base"
                self._add_agent_meshes_to_body(parts[0], agent_root, base_label)

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

                # Check for corresponding part
                part_index = i + 1
                if part_index < len(parts):
                    label = (
                        labels[part_index]
                        if labels and part_index < len(labels)
                        else f"agent_link_{i}"
                    )
                    self._add_agent_meshes_to_body(parts[part_index], child_body, label)
                else:
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

            # Attach remaining parts to base to avoid losing geometry
            for i in range(len(agent_joints) + 1, len(parts)):
                label = labels[i] if labels and i < len(labels) else f"agent_part_{i}"
                self._add_agent_meshes_to_body(parts[i], agent_root, label)

        # Add a placeholder for automatic actuators if not already handled
        # This allows mixed manual/auto but we mostly care about pure auto for now
        self._inject_auto_actuators(agent_root, actuator_root)

    def _inject_auto_actuators(self, agent_root: ET.Element, actuator_root: ET.Element):
        """Scan for stator/rotor pairs and inject joints and actuators."""
        # Find all geoms under worldbody specifically for the agent
        # (Though they are already in agent_root)

        # We need to find geoms with "stator" and "rotor" in their names or labels.
        # However, the geoms were added with names based on 'label'.
        # Let's look at how geoms are named in _add_agent_meshes_to_body.
        pass  # We will instead do it during mesh processing

    def _add_agent_meshes_to_body(
        self, compound: Compound, body_element: ET.Element, prefix: str
    ):
        """Helper to add meshes from a compound to a specific body element."""
        print(f"DEBUG: STARTing _add_agent_meshes_to_body with prefix {prefix}")
        print(
            f"DEBUG: Compound type: {type(compound)}, label: '{getattr(compound, 'label', '')}'"
        )

        # We iterate over top-level children of the compound to find motors
        # If it's a single solid, it will have no children but solids() will have 1.

        children = getattr(compound, "children", [])
        print(f"DEBUG: Children count: {len(children)}")
        if not children and isinstance(compound, Solid):
            children = [compound]

        # If the compound itself is what we want to process (e.g. from the test)
        # but it's wrapped in another compound, we might need to look deeper.
        # For now, let's just make it handle the case where 'compound' IS the motor.

        # Check if the compound itself has the labels
        stator_solid = None
        rotor_solid = None
        for child in children:
            label = getattr(child, "label", "").lower()
            if "stator" in label:
                stator_solid = list(child.solids())[0] if list(child.solids()) else None
            elif "rotor" in label:
                rotor_solid = list(child.solids())[0] if list(child.solids()) else None

        # If found in children, processed it as a motor
        if stator_solid and rotor_solid:
            self._process_motor_element(
                compound, stator_solid, rotor_solid, body_element, prefix
            )
            return

        # If not, iterate through children and see if any of them are motors
        remaining_solids = list(compound.solids())
        processed_solids = set()

        for i, child in enumerate(children):
            c_stator = None
            c_rotor = None
            child_children = getattr(child, "children", [])
            print(f"DEBUG: Child {i} has {len(child_children)} children")
            for cc in child_children:
                cc_label = getattr(cc, "label", "").lower()
                print(f"DEBUG: Grandchild label: '{cc_label}'")
                if "stator" in cc_label:
                    c_stator = list(cc.solids())[0] if list(cc.solids()) else None
                elif "rotor" in cc_label:
                    c_rotor = list(cc.solids())[0] if list(cc.solids()) else None

            if c_stator and c_rotor:
                print(f"DEBUG: Motor detected in child {i}!")
                c_prefix = f"{prefix}_p{i}"
                self._process_motor_element(
                    child, c_stator, c_rotor, body_element, c_prefix
                )
                for s in child.solids():
                    processed_solids.add(id(s))
            else:
                # If child is not a motor, but it's a compound, we should probably recurse
                # or just let the fallback handle its solids.
                pass

        # Fallback: add all unprocessed solids
        for i, solid in enumerate(remaining_solids):
            if id(solid) not in processed_solids:
                label = f"{prefix}_{i}"
                self._add_solid_to_body(solid, body_element, label)

    def _process_motor_element(
        self, motor_compound, stator_solid, rotor_solid, body_element, prefix
    ):
        """Internal helper to add motor components and joints."""
        # 1. Add Stator to current body
        stator_label = f"{prefix}_stator"
        self._add_solid_to_body(stator_solid, body_element, stator_label)

        # 2. Add Rotor in a child body
        rotor_center = rotor_solid.center()
        center_str = f"{rotor_center.X} {rotor_center.Y} {rotor_center.Z}"

        rotor_body = ET.SubElement(
            body_element, "body", name=f"{prefix}_rotor_body", pos="0 0 0"
        )

        # Get simulation metadata
        sim_meta = {"joint_type": "hinge", "joint_axis": "0 0 1", "gear": "10"}
        # Try to find a part name in any label
        all_labels = [getattr(motor_compound, "label", "")] + [
            getattr(s, "label", "") for s in motor_compound.solids()
        ]
        label_text = " ".join(all_labels).lower()

        if "nema17" in label_text:
            meta = get_description("Nema17")
            sim_meta.update(meta.get("simulation", {}))
        elif "nema23" in label_text:
            meta = get_description("Nema23")
            sim_meta.update(meta.get("simulation", {}))

        j_name = f"joint_{prefix}"
        ET.SubElement(
            rotor_body,
            "joint",
            name=j_name,
            type=sim_meta["joint_type"],
            pos=center_str,
            axis=sim_meta["joint_axis"],
            damping=str(sim_meta.get("damping", 0.1)),
        )

        rotor_label = f"{prefix}_rotor"
        self._add_solid_to_body(rotor_solid, rotor_body, rotor_label)

        # 3. Add Actuator
        actuator_root = self.root.find("actuator")
        if actuator_root is None:
            actuator_root = ET.SubElement(self.root, "actuator")

        ET.SubElement(
            actuator_root,
            "motor",
            name=f"motor_{prefix}",
            joint=j_name,
            gear=str(sim_meta["gear"]),
        )

        # Add any other solids in the motor_compound to the base body
        for i, s in enumerate(motor_compound.solids()):
            if id(s) != id(stator_solid) and id(s) != id(rotor_solid):
                self._add_solid_to_body(s, body_element, f"{prefix}_m_{i}")

    def _add_solid_to_body(self, solid: Solid, body_element: ET.Element, label: str):
        """Exports a solid to STL and adds it as a geom to the body."""
        stl_data = MeshProcessor.export_stl(solid)
        mesh_filename = f"{label}.stl"

        if self.asset_dir:
            asset_path = Path(self.asset_dir)
            file_path = asset_path / mesh_filename
            file_path.write_bytes(stl_data)

        # Ensure mesh asset exists
        if self.asset.find(f"mesh[@name='{label}']") is None:
            ET.SubElement(self.asset, "mesh", name=label, file=mesh_filename)

        ET.SubElement(body_element, "geom", type="mesh", mesh=label, rgba="0 0 1 1")
