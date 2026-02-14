from __future__ import annotations

import json
import logging
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from pathlib import Path
from typing import TYPE_CHECKING, Any
from xml.dom import minidom

import trimesh
from build123d import Compound, Solid, export_stl

from shared.cots.parts.motors import ServoMotor
from shared.simulation.backends import SimulatorBackendType
from worker.utils.mesh_utils import tetrahedralize

if TYPE_CHECKING:
    from shared.models.schemas import MovingPart, ObjectivesYaml

logger = logging.getLogger(__name__)


class MeshProcessor:
    """Handles conversion from build123d geometry to physics-ready meshes.

    Per architecture spec:
    - Exports to OBJ format (less bulky than STL)
    - Recenters parts to origin before export (position comes from MJCF body)
    - Validates watertightness for all meshes
    """

    def process_geometry(
        self,
        part: Solid | Compound,
        filepath: Path,
        decompose: bool = True,
        use_vhacd: bool = False,
    ) -> list[Path]:
        """Converts a build123d object to OBJ file(s).

        Args:
            part: The build123d geometry to convert
            filepath: Output path (will be changed to .obj extension)
            decompose: Whether to compute convex hull for physics
            use_vhacd: Whether to use V-HACD decomposition for concave shapes

        Returns:
            List of output file paths

        Raises:
            ValueError: If mesh is not watertight
        """
        # Ensure the directory exists
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # Force .obj extension for output
        filepath = filepath.with_suffix(".obj")

        # Export build123d object to a temporary STL file
        # (build123d doesn't have native OBJ export, so we convert via trimesh)
        temp_stl = filepath.with_suffix(".tmp.stl")
        export_stl(part, str(temp_stl))

        output_paths = []
        try:
            mesh = trimesh.load(str(temp_stl))

            # If trimesh loaded a Scene, merge it into a single mesh
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)

            # Recenter mesh to origin (position will come from MJCF body pos)
            mesh = self._recenter_mesh(mesh)

            # Validate watertightness
            self._validate_watertight(mesh, filepath.name)

            if decompose:
                if use_vhacd:
                    try:
                        decomposed = trimesh.decomposition.convex_decomposition(mesh)
                        if isinstance(decomposed, list) and len(decomposed) > 1:
                            for i, dm in enumerate(decomposed):
                                sub_path = filepath.with_name(
                                    f"{filepath.stem}_{i}.obj"
                                )
                                dm.export(str(sub_path), file_type="obj")
                                output_paths.append(sub_path)
                            return output_paths
                    except Exception:
                        # Fallback to single convex hull if VHACD fails
                        pass

                mesh = self.compute_convex_hull(mesh)

            # Export as OBJ format
            mesh.export(str(filepath), file_type="obj")
            output_paths.append(filepath)
        finally:
            if temp_stl.exists():
                temp_stl.unlink()

        return output_paths

    def _recenter_mesh(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """Recenter mesh to origin (0,0,0) based on its centroid."""
        centroid = mesh.centroid
        mesh.apply_translation(-centroid)
        return mesh

    def _validate_watertight(self, mesh: trimesh.Trimesh, name: str) -> None:
        """Validate that mesh is watertight (required per architecture spec)."""
        if not mesh.is_watertight:
            raise ValueError(
                f"Mesh '{name}' is not watertight. "
                "All meshes must be watertight for physics simulation."
            )

    def compute_convex_hull(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        """Computes the convex hull of a mesh for better physics stability."""
        return mesh.convex_hull


class SceneCompiler:
    """Generates MJCF XML for MuJoCo simulation."""

    def __init__(self, model_name: str = "simulation_scene"):
        self.root = ET.Element("mujoco", model=model_name)

        # Basic configuration
        self.compiler = ET.SubElement(
            self.root, "compiler", angle="degree", coordinate="local", assetdir="assets"
        )
        ET.SubElement(self.root, "option", integrator="RK4", timestep="0.002")
        self.actuators = ET.SubElement(self.root, "actuator")

        # Visual assets and lighting
        visual = ET.SubElement(self.root, "visual")
        ET.SubElement(
            visual,
            "headlight",
            diffuse="0.6 0.6 0.6",
            ambient="0.3 0.3 0.3",
            specular="0 0 0",
        )

        self.assets = ET.SubElement(self.root, "asset")
        # Default floor texture and material
        ET.SubElement(
            self.assets,
            "texture",
            name="grid",
            type="2d",
            builtin="checker",
            rgb1=".1 .2 .3",
            rgb2=".2 .3 .4",
            width="300",
            height="300",
        )
        ET.SubElement(
            self.assets,
            "material",
            name="grid",
            texture="grid",
            texrepeat="1 1",
            texuniform="true",
        )

        self.worldbody = ET.SubElement(self.root, "worldbody")
        ET.SubElement(
            self.worldbody, "light", pos="0 0 3", dir="0 0 -1", directional="true"
        )
        ET.SubElement(
            self.worldbody,
            "geom",
            name="floor",
            size="10 10 0.1",
            type="plane",
            material="grid",
        )

        # Equality constraints
        self.equality = ET.SubElement(self.root, "equality")

    def add_mesh_asset(self, name: str, file_name: str):
        """Registers a mesh file in the MJCF assets."""
        ET.SubElement(self.assets, "mesh", name=name, file=file_name)

    def add_site(
        self,
        name: str,
        pos: list[float],
        parent_body_name: str | None = None,
        size: float = 0.001,
        rgba: str | None = None,
    ):
        """Adds a site to a body or worldbody."""
        parent = self.worldbody
        if parent_body_name:
            # Simple linear search for the body in worldbody
            for body in self.worldbody.iter("body"):
                if body.get("name") == parent_body_name:
                    parent = body
                    break

        attrs = {"name": name, "pos": " ".join(map(str, pos)), "size": str(size)}
        if rgba:
            attrs["rgba"] = rgba
        ET.SubElement(parent, "site", **attrs)

    def add_spatial_tendon(
        self,
        name: str,
        site_names: list[str],
        width: float = 0.002,
        rgba: str = "0.1 0.1 0.8 1",
        limited: bool = False,
        range: list[float] | None = None,
    ):
        """Adds a spatial tendon connecting a sequence of sites."""
        if not hasattr(self, "tendon_element"):
            self.tendon_element = ET.SubElement(self.root, "tendon")

        attrs = {"name": name, "width": str(width), "rgba": rgba}
        if limited and range:
            attrs["limited"] = "true"
            attrs["range"] = " ".join(map(str, range))

        spatial = ET.SubElement(self.tendon_element, "spatial", **attrs)
        for site_name in site_names:
            ET.SubElement(spatial, "site", site=site_name)

    def add_weld(self, body1: str, body2: str):
        """Adds a weld constraint between two bodies."""
        ET.SubElement(self.equality, "weld", body1=body1, body2=body2)

    def add_body(
        self,
        name: str,
        mesh_names: list[str] | None = None,
        pos: list[float] | None = None,
        euler: list[float] | None = None,
        is_zone: bool = False,
        zone_type: str | None = None,
        zone_size: list[float] | None = None,
        is_fixed: bool = False,
        joint_type: str | None = None,
        joint_axis: list[float] | None = None,
        joint_range: list[float] | None = None,
    ):
        """Adds a body to the worldbody. Can be a physical mesh or a logical zone.

        Args:
            name: Body identifier
            mesh_names: List of mesh asset names for this body
            pos: Position [x, y, z], defaults to [0, 0, 0]
            euler: Euler angles [rx, ry, rz], defaults to [0, 0, 0]
            is_zone: Whether this is a logical zone (goal/forbid)
            zone_type: "goal" or "forbid" for zones
            zone_size: Half-extents for zone box
            is_fixed: If True, part is fixed (no free joint added)
            joint_type: Optional joint type (e.g., "hinge", "slide").
            joint_axis: Optional joint axis [x, y, z].
            joint_range: Optional joint limits [min, max].
        """
        if pos is None:
            pos = [0, 0, 0]
        if euler is None:
            euler = [0, 0, 0]

        body = ET.SubElement(
            self.worldbody, "body", name=name, pos=" ".join(map(str, pos))
        )
        if any(v != 0 for v in euler):
            body.set("euler", " ".join(map(str, euler)))

        if is_zone:
            # Zone Logic (T005): goal = green, forbid = red
            rgba = "0 1 0 0.3" if zone_type == "goal" else "1 0 0 0.3"
            size_str = " ".join(map(str, zone_size)) if zone_size else "0.05"
            ET.SubElement(body, "site", name=name, type="box", size=size_str, rgba=rgba)
        else:
            if mesh_names:
                for mesh_name in mesh_names:
                    ET.SubElement(body, "geom", type="mesh", mesh=mesh_name)

            # Handle joints
            if is_fixed:
                return

            if joint_type:
                # Add specific joint
                joint_attrs = {"type": joint_type, "name": f"{name}_joint"}
                if joint_axis:
                    joint_attrs["axis"] = " ".join(map(str, joint_axis))
                if joint_range:
                    joint_attrs["range"] = " ".join(map(str, joint_range))
                ET.SubElement(body, "joint", **joint_attrs)
            else:
                # Default to free joint
                logger.warning(
                    f"Adding free joint to body '{name}'. "
                    "This part will fall if not supported. "
                    "Use 'fixed=True' or 'constraint' attribute to secure it."
                )
                ET.SubElement(body, "joint", type="free")

    def add_actuator(
        self,
        name: str,
        joint: str,
        kp: float | None = None,
        kv: float | None = None,
        forcerange: tuple[float, float] | None = None,
        actuator_type: str = "position",
    ):
        """Adds an actuator (motor/servo) to control a joint.

        Args:
            name: Unique name for the actuator.
            joint: Name of the joint to control.
            kp: Proportional gain for position control.
            kv: Derivative gain for position control.
            forcerange: (min, max) torque limits in N·m. From COTS servo specs.
            actuator_type: "position" for servos, "motor" for direct torque.
        """
        # Default gains if not derived
        final_kp = kp if kp is not None else 10.0
        final_kv = kv if kv is not None else 1.0
        final_forcerange = forcerange

        # Try to derive from COTS if missing parameters
        if kp is None or forcerange is None:
            # Attempt to extract COTS model name from actuator name
            # Assuming format "name" or "Servo_SG90_..."
            # Simple heuristic: check if any COTS model name is in the string
            found_model = None
            for model_name, data in ServoMotor.motor_data.items():
                if model_name in name or model_name in joint:
                    found_model = model_name
                    # Parse data
                    torque = data["torque_nm"]

                    if forcerange is None:
                        final_forcerange = (-torque, torque)

                    if kp is None:
                        # Heuristic: kp = torque / saturation_error (rad)
                        # Assume saturation at ~0.2 rad (~11 deg)
                        saturation_error = 0.2
                        final_kp = torque / saturation_error

                        if kv is None:
                            # Critical damping approx or small damping
                            final_kv = final_kp * 0.1
                    break

        attrs = {
            "name": name,
            "joint": joint,
            "kp": str(final_kp),
            "kv": str(final_kv),
        }
        if final_forcerange is not None:
            attrs["forcerange"] = f"{final_forcerange[0]} {final_forcerange[1]}"

        ET.SubElement(self.actuators, actuator_type, **attrs)

    def save(self, path: Path):
        """Saves the MJCF XML to a file."""
        xml_str = ET.tostring(self.root, encoding="utf-8")
        pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="  ")
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w") as f:
            f.write(pretty_xml)


class SimulationBuilderBase(ABC):
    """Abstract base class for simulation builders."""

    def __init__(self, output_dir: Path, use_vhacd: bool = False):
        self.output_dir = Path(output_dir)
        self.assets_dir = self.output_dir / "assets"
        self.processor = MeshProcessor()
        self.use_vhacd = use_vhacd

    @abstractmethod
    def build_from_assembly(
        self,
        assembly: Compound,
        objectives: ObjectivesYaml | None = None,
        moving_parts: list[MovingPart] | None = None,
        electronics: Any | None = None,
    ) -> Path:
        """Converts an assembly of parts into a simulation scene."""
        pass


class MuJoCoSimulationBuilder(SimulationBuilderBase):
    """Orchestrates the conversion of build123d assemblies to MuJoCo scenes."""

    def __init__(self, output_dir: Path, use_vhacd: bool = False):
        super().__init__(output_dir, use_vhacd)
        self.compiler = SceneCompiler()

    def build_from_assembly(
        self,
        assembly: Compound,
        objectives: ObjectivesYaml | None = None,
        moving_parts: list[MovingPart] | None = None,
        electronics: Any | None = None,
    ) -> Path:
        """Converts an assembly of parts into a MuJoCo scene.xml and associated STLs."""
        self.assets_dir.mkdir(parents=True, exist_ok=True)

        weld_constraints = []
        body_locations = {}  # name -> (pos, euler)

        # 1. Add zones from objectives if provided
        if objectives:
            # Add Goal Zone
            gz = objectives.objectives.goal_zone
            # Calculate center and half-extents
            gz_pos = [
                (gz.min[0] + gz.max[0]) / 2,
                (gz.min[1] + gz.max[1]) / 2,
                (gz.min[2] + gz.max[2]) / 2,
            ]
            gz_size = [
                (gz.max[0] - gz.min[0]) / 2,
                (gz.max[1] - gz.min[1]) / 2,
                (gz.max[2] - gz.min[2]) / 2,
            ]
            self.compiler.add_body(
                name="zone_goal",
                is_zone=True,
                zone_type="goal",
                zone_size=gz_size,
                pos=gz_pos,
            )

            # Add Forbid Zones
            for i, fz in enumerate(objectives.objectives.forbid_zones):
                fz_pos = [
                    (fz.min[0] + fz.max[0]) / 2,
                    (fz.min[1] + fz.max[1]) / 2,
                    (fz.min[2] + fz.max[2]) / 2,
                ]
                fz_size = [
                    (fz.max[0] - fz.min[0]) / 2,
                    (fz.max[1] - fz.min[1]) / 2,
                    (fz.max[2] - fz.min[2]) / 2,
                ]
                self.compiler.add_body(
                    name=f"zone_forbid_{i}_{fz.name}",
                    is_zone=True,
                    zone_type="forbid",
                    zone_size=fz_size,
                    pos=fz_pos,
                )

        # 2. Add parts from assembly
        # Handle both Compound (with children) and single Solid
        children = getattr(assembly, "children", [])
        if not children:
            # If it's a single shape without children, treat it as the only child
            children = [assembly]

        for i, child in enumerate(children):
            # Try to get label, fallback to indexed name
            label = getattr(child, "label", None) or f"part_{i}"

            # Check for constraints metadata
            constraint = getattr(child, "constraint", None)
            joint_type = None
            joint_axis = None
            joint_range = None

            if constraint and isinstance(constraint, str):
                # Support multiple constraints separated by semicolon if needed,
                # but for now assume single constraint string
                if constraint.startswith("weld:"):
                    target = constraint.split(":", 1)[1]
                    weld_constraints.append((label, target))
                elif constraint.startswith(("hinge", "slide")):
                    # Parse joint constraint: type:axis:range
                    # e.g., "hinge:z", "hinge:x:range=-45,45", "slide:0,1,0"
                    parts = constraint.split(":")
                    joint_type = parts[0]

                    if len(parts) > 1:
                        axis_str = parts[1]
                        # Handle shorthand axis
                        if axis_str == "x":
                            joint_axis = [1, 0, 0]
                        elif axis_str == "y":
                            joint_axis = [0, 1, 0]
                        elif axis_str == "z":
                            joint_axis = [0, 0, 1]
                        elif "," in axis_str:
                            try:
                                joint_axis = [float(x) for x in axis_str.split(",")]
                            except ValueError:
                                logger.warning(f"Invalid axis format: {axis_str}")
                        elif " " in axis_str:
                            try:
                                joint_axis = [float(x) for x in axis_str.split()]
                            except ValueError:
                                logger.warning(f"Invalid axis format: {axis_str}")

                    # Check for range
                    for part in parts[2:]:
                        if part.startswith("range="):
                            range_str = part.split("=")[1]
                            try:
                                if "," in range_str:
                                    joint_range = [
                                        float(x) for x in range_str.split(",")
                                    ]
                                else:
                                    joint_range = [float(x) for x in range_str.split()]
                            except ValueError:
                                logger.warning(f"Invalid range format: {range_str}")

            # Position from build123d object
            pos = [
                child.location.position.X,
                child.location.position.Y,
                child.location.position.Z,
            ]
            # Convert orientation to euler (degrees) for MJCF
            euler = [
                child.location.orientation.X,
                child.location.orientation.Y,
                child.location.orientation.Z,
            ]

            if label.startswith("zone_"):
                zone_type = "goal" if "goal" in label else "forbid"
                # Extract bounding box size for the zone
                bb = child.bounding_box()
                # MuJoCo box size is half-extents
                zone_size = [bb.size.X / 2, bb.size.Y / 2, bb.size.Z / 2]

                self.compiler.add_body(
                    name=label,
                    is_zone=True,
                    zone_type=zone_type,
                    zone_size=zone_size,
                    pos=pos,
                    euler=euler,
                )
            else:
                # Use .obj extension for mesh files (per architecture spec)
                mesh_filename_base = f"{label}.obj"
                mesh_path_base = self.assets_dir / mesh_filename_base

                # Process geometry and save STL(s)
                saved_paths = self.processor.process_geometry(
                    child, mesh_path_base, use_vhacd=self.use_vhacd
                )

                mesh_names = []
                for j, path in enumerate(saved_paths):
                    mesh_name = f"{label}_{j}" if len(saved_paths) > 1 else label
                    self.compiler.add_mesh_asset(name=mesh_name, file_name=path.name)
                    mesh_names.append(mesh_name)

                # Add body with all generated meshes as geoms
                # Check if part is marked as fixed (no free joint in simulation)
                is_fixed = getattr(child, "fixed", False)
                self.compiler.add_body(
                    name=label,
                    mesh_names=mesh_names,
                    pos=pos,
                    euler=euler,
                    is_fixed=is_fixed,
                    joint_type=joint_type,
                    joint_axis=joint_axis,
                    joint_range=joint_range,
                )
                body_locations[label] = (pos, euler)

        # Apply collected constraints
        for body1, body2 in weld_constraints:
            self.compiler.add_weld(body1, body2)

        # 3. Add actuators for moving parts (T011)
        if moving_parts:
            for mp in moving_parts:
                if mp.type == "motor":
                    # We assume the joint name follows the convention {part_name}_joint
                    # which is what add_body uses.
                    self.compiler.add_actuator(
                        name=mp.part_name,
                        joint=f"{mp.part_name}_joint",
                        actuator_type="position",  # Defaulting to position for servos
                    )

        # 4. Add Electronics (Wires/Tendons)
        if electronics and hasattr(electronics, "wiring"):
            for wire in electronics.wiring:
                if not getattr(wire, "routed_in_3d", False):
                    continue

                site_names = []
                waypoints = getattr(wire, "waypoints", [])

                if waypoints:
                    for j, pt in enumerate(waypoints):
                        site_name = f"site_{wire.wire_id}_{j}"
                        # For now, treat all waypoints as world-relative
                        # A more advanced version would find the closest body
                        self.compiler.add_site(name=site_name, pos=list(pt))
                        site_names.append(site_name)
                else:
                    # Fallback: connect from/to components directly
                    from_comp = wire.from_terminal.component
                    to_comp = wire.to_terminal.component

                    for comp_id in [from_comp, to_comp]:
                        site_name = f"site_{comp_id}_{wire.wire_id}"
                        pos = [0, 0, 0]
                        parent = None
                        if comp_id in body_locations:
                            parent = comp_id
                        self.compiler.add_site(
                            name=site_name, pos=pos, parent_body_name=parent
                        )
                        site_names.append(site_name)

                if len(site_names) >= 2:
                    # Add tendon
                    # Width scaling: AWG 10 is ~2.5mm, AWG 20 is ~0.8mm
                    # Simple linear approx:
                    width = 0.001 + (20 - wire.gauge_awg) * 0.0001
                    self.compiler.add_spatial_tendon(
                        name=wire.wire_id,
                        site_names=site_names,
                        width=max(0.0005, width),
                    )

        scene_path = self.output_dir / "scene.xml"
        self.compiler.save(scene_path)
        return scene_path


# Alias for backward compatibility
SimulationBuilder = MuJoCoSimulationBuilder


class GenesisSimulationBuilder(SimulationBuilderBase):
    """Orchestrates the conversion of build123d assemblies to Genesis scenes."""

    def build_from_assembly(
        self,
        assembly: Compound,
        objectives: ObjectivesYaml | None = None,
        moving_parts: list[MovingPart] | None = None,
        electronics: Any | None = None,
    ) -> Path:
        """Converts an assembly of parts into a Genesis scene descriptor (JSON)."""
        self.assets_dir.mkdir(parents=True, exist_ok=True)

        scene_data = {"entities": [], "fluids": [], "objectives": []}

        # 1. Add zones from objectives
        if objectives:
            # Add Goal Zone
            gz = objectives.objectives.goal_zone
            gz_pos = [(gz.min[i] + gz.max[i]) / 2 for i in range(3)]
            gz_size = [(gz.max[i] - gz.min[i]) / 2 for i in range(3)]
            scene_data["entities"].append(
                {
                    "name": "zone_goal",
                    "type": "box",
                    "pos": gz_pos,
                    "size": gz_size,
                    "is_zone": True,
                    "zone_type": "goal",
                }
            )

            # Add Forbid Zones
            for i, fz in enumerate(objectives.objectives.forbid_zones):
                fz_pos = [(fz.min[j] + fz.max[j]) / 2 for j in range(3)]
                fz_size = [(fz.max[j] - fz.min[j]) / 2 for j in range(3)]
                scene_data["entities"].append(
                    {
                        "name": f"zone_forbid_{i}_{fz.name}",
                        "type": "box",
                        "pos": fz_pos,
                        "size": fz_size,
                        "is_zone": True,
                        "zone_type": "forbid",
                    }
                )

        # 2. Add parts from assembly
        children = getattr(assembly, "children", [])
        if not children:
            children = [assembly]

        # Load manufacturing config to check for deformable materials
        from worker.workbenches.config import load_config

        mfg_config = load_config()

        for i, child in enumerate(children):
            label = getattr(child, "label", None) or f"part_{i}"
            material_id = getattr(child, "material_id", "aluminum_6061")

            # Check if deformable
            is_deformable = False
            if objectives and objectives.physics and objectives.physics.fem_enabled:
                is_deformable = True
            else:
                mat_def = mfg_config.materials.get(material_id)
                if mat_def and mat_def.material_class in ["soft", "elastomer"]:
                    is_deformable = True

            # Position and orientation
            pos = [
                child.location.position.X,
                child.location.position.Y,
                child.location.position.Z,
            ]
            euler = [
                child.location.orientation.X,
                child.location.orientation.Y,
                child.location.orientation.Z,
            ]

            mesh_filename = f"{label}.obj"
            mesh_path = self.assets_dir / mesh_filename

            # Process geometry (save as OBJ for rigid, or intermediate STL for soft)
            self.processor.process_geometry(child, mesh_path, decompose=False)

            entity_info = {
                "name": label,
                "pos": pos,
                "euler": euler,
                "material_id": material_id,
            }

            if is_deformable:
                msh_path = mesh_path.with_suffix(".msh")
                # process_geometry saves to .obj, but also produces a .tmp.stl internally.
                # We might need to export STL explicitly for tetrahedralize.
                stl_path = mesh_path.with_suffix(".stl")
                export_stl(child, str(stl_path))
                tetrahedralize(stl_path, msh_path)
                entity_info["type"] = "soft_mesh"
                entity_info["file"] = str(msh_path.relative_to(self.output_dir))
            else:
                entity_info["type"] = "mesh"
                entity_info["file"] = str(mesh_path.relative_to(self.output_dir))

            scene_data["entities"].append(entity_info)

        # 3. Add Fluids (MPM) from objectives if defined
        if objectives and hasattr(objectives, "fluids") and objectives.fluids:
            for fluid in objectives.fluids:
                scene_data["fluids"].append(fluid.model_dump())

        # 4. Add Fluid Objectives
        if (
            objectives
            and hasattr(objectives.objectives, "fluid_objectives")
            and objectives.objectives.fluid_objectives
        ):
            for fo in objectives.objectives.fluid_objectives:
                scene_data["objectives"].append(fo.model_dump())

        scene_path = self.output_dir / "scene.json"
        with open(scene_path, "w") as f:
            json.dump(scene_data, f, indent=2)

        return scene_path
