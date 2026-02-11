from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TYPE_CHECKING
from xml.dom import minidom

import trimesh
from build123d import Compound, Solid, export_stl

from shared.cots.parts.motors import ServoMotor

if TYPE_CHECKING:
    from shared.models.schemas import ObjectivesYaml

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
            # Add a free joint so the body can move (unless fixed)
            if not is_fixed:
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


class SimulationBuilder:
    """Orchestrates the conversion of build123d assemblies to MuJoCo scenes."""

    def __init__(self, output_dir: Path, use_vhacd: bool = False):
        self.output_dir = Path(output_dir)
        self.assets_dir = self.output_dir / "assets"
        self.processor = MeshProcessor()
        self.compiler = SceneCompiler()
        self.use_vhacd = use_vhacd

    def build_from_assembly(
        self, assembly: Compound, objectives: "ObjectivesYaml" | None = None
    ) -> Path:
        """Converts an assembly of parts into a MuJoCo scene.xml and associated STLs."""
        self.assets_dir.mkdir(parents=True, exist_ok=True)

        weld_constraints = []

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
        for i, child in enumerate(assembly.children):
            # Try to get label, fallback to indexed name
            label = getattr(child, "label", None) or f"part_{i}"

            # Check for constraints metadata
            constraint = getattr(child, "constraint", None)
            if constraint and isinstance(constraint, str):
                if constraint.startswith("weld:"):
                    target = constraint.split(":", 1)[1]
                    weld_constraints.append((label, target))

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
                )

        # Apply collected constraints
        for body1, body2 in weld_constraints:
            self.compiler.add_weld(body1, body2)

        scene_path = self.output_dir / "scene.xml"
        self.compiler.save(scene_path)
        return scene_path
