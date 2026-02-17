from __future__ import annotations

import json
import logging
import tempfile
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from pathlib import Path
from typing import TYPE_CHECKING, Any
from xml.dom import minidom

import trimesh
from build123d import Compound, Solid, export_stl

# YACV removed in favor of custom trimesh-based export capable of preserving topology
# try:
#     from yacv.exporter import export_all
# except ImportError:
#     export_all = None

if TYPE_CHECKING:
    from shared.models.schemas import (
        MovingPart,
        ObjectivesYaml,
    )

from pydantic import BaseModel, ConfigDict


class AssemblyPartData(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    label: str
    part: Solid | Compound | Any
    pos: list[float]
    euler: list[float]
    is_fixed: bool = False
    joint_type: str | None = None
    joint_axis: list[float] | None = None
    joint_range: list[float] | None = None
    material_id: str | None = None
    cots_id: str | None = None
    is_electronics: bool = False
    is_zone: bool = False
    zone_type: str | None = None
    zone_size: list[float] | None = None
    constraint: str | None = None


class CommonAssemblyTraverser:
    """Unifies assembly traversal and metadata resolution."""

    @staticmethod
    def traverse(
        assembly: Compound, electronics: Any | None = None
    ) -> list[AssemblyPartData]:
        children = getattr(assembly, "children", [])
        if not children:
            children = [assembly]

        parts_data = []
        for i, child in enumerate(children):
            label = getattr(child, "label", None) or f"part_{i}"

            pos, euler = CommonAssemblyTraverser._resolve_location(child)
            meta = CommonAssemblyTraverser._resolve_part_metadata(child)
            zone_info = CommonAssemblyTraverser._detect_zone(child, label)
            is_electronics = CommonAssemblyTraverser._map_electronics(
                label, electronics
            )

            parts_data.append(
                AssemblyPartData(
                    label=label,
                    part=child,
                    pos=pos,
                    euler=euler,
                    is_fixed=meta["is_fixed"],
                    material_id=meta["material_id"],
                    cots_id=meta["cots_id"],
                    joint_type=meta["joint_type"],
                    joint_axis=meta["joint_axis"],
                    joint_range=meta["joint_range"],
                    is_electronics=is_electronics,
                    is_zone=zone_info["is_zone"],
                    zone_type=zone_info["type"],
                    zone_size=zone_info["size"],
                    constraint=getattr(child, "constraint", None),
                )
            )
        return parts_data

    @staticmethod
    def _resolve_location(child: Any) -> tuple[list[float], list[float]]:
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
        return pos, euler

    @staticmethod
    def _resolve_part_metadata(child: Any) -> dict[str, Any]:
        from shared.models.schemas import CompoundMetadata, PartMetadata

        metadata = getattr(child, "metadata", None)
        if metadata is None:
            # Check for legacy zone prefix - zones are handled separately
            label = getattr(child, "label", "")
            if label.startswith("zone_"):
                return {
                    "is_fixed": True,
                    "material_id": None,
                    "cots_id": None,
                    "joint_type": None,
                    "joint_axis": None,
                    "joint_range": None,
                }

            raise ValueError(
                f"Part '{label or 'unknown'}' is missing required metadata. "
                "Every part must have a .metadata attribute "
                "(PartMetadata or CompoundMetadata)."
            )

        if isinstance(metadata, dict):
            try:
                metadata = PartMetadata(**metadata)
            except Exception:
                try:
                    metadata = CompoundMetadata(**metadata)
                except Exception as e:
                    label_str = getattr(child, "label", "unknown")
                    raise ValueError(
                        f"Invalid metadata for part '{label_str}': {e}"
                    ) from e

        joint_type, joint_axis, joint_range = None, None, None
        if metadata.joint:
            joint_type = metadata.joint.type
            joint_axis = list(metadata.joint.axis)
            joint_range = list(metadata.joint.range) if metadata.joint.range else None

        return {
            "is_fixed": metadata.is_fixed,
            "material_id": getattr(metadata, "material_id", None),
            "cots_id": getattr(metadata, "cots_id", None),
            "joint_type": joint_type,
            "joint_axis": joint_axis,
            "joint_range": joint_range,
        }

    @staticmethod
    def _detect_zone(child: Any, label: str) -> dict[str, Any]:
        is_zone = False
        zone_type = None
        zone_size = None
        if label.startswith("zone_"):
            is_zone = True
            zone_type = "goal" if "goal" in label else "forbid"
            bb = child.bounding_box()
            zone_size = [bb.size.X / 2, bb.size.Y / 2, bb.size.Z / 2]
        return {"is_zone": is_zone, "type": zone_type, "size": zone_size}

    @staticmethod
    def _map_electronics(label: str, electronics: Any | None) -> bool:
        if electronics and hasattr(electronics, "components"):
            for comp in electronics.components:
                if getattr(comp, "assembly_part_ref", None) == label:
                    return True
        return False


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
        """Converts a build123d object to OBJ and GLB file(s).

        OBJ is used for physics simulation (MuJoCo/Genesis),
        while GLB is used for efficient frontend visualization.

        Args:
            part: The build123d geometry to convert
            filepath: Output base path
            decompose: Whether to compute convex hull for physics
            use_vhacd: Whether to use V-HACD decomposition for concave shapes

        Returns:
            List of output file paths (both .obj and .glb)

        Raises:
            ValueError: If mesh is not watertight
        """
        # Ensure the directory exists
        filepath.parent.mkdir(parents=True, exist_ok=True)

        # Export build123d object to a temporary STL file
        temp_stl = filepath.with_suffix(".tmp.stl")
        export_stl(part, temp_stl)

        output_paths = []
        try:
            mesh = trimesh.load(temp_stl)

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
                                # Export both OBJ and GLB for each decomposed part
                                obj_sub = filepath.with_name(f"{filepath.stem}_{i}.obj")
                                glb_sub = filepath.with_name(f"{filepath.stem}_{i}.glb")
                                dm.export(obj_sub, file_type="obj")
                                dm.export(glb_sub, file_type="glb")
                                output_paths.extend([obj_sub, glb_sub])
                            return output_paths
                    except Exception:
                        # Fallback to single convex hull if VHACD fails
                        pass

                mesh = self.compute_convex_hull(mesh)

            # Export in both formats
            obj_path = filepath.with_suffix(".obj")
            glb_path = filepath.with_suffix(".glb")

            mesh.export(obj_path, file_type="obj")

            # Custom topology-preserving export for frontend viewer
            try:
                self.export_topology_glb(part, glb_path)
            except Exception as e:
                logger.warning("topology_export_failed", error=str(e))
                # Fallback to simple mesh export
                mesh.export(glb_path, file_type="glb")

            output_paths.extend([obj_path, glb_path])
        finally:
            if temp_stl.exists():
                temp_stl.unlink()

        return output_paths

    def export_topology_glb(self, part: Solid | Compound, filepath: Path):
        """
        Exports the part to GLB with separate meshes for faces to allow selection in UI.
        """
        scene = trimesh.Scene()

        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)

            # 1. Export Faces
            # access faces via topological properties
            faces = part.faces()
            for i, face in enumerate(faces):
                fname = tmp_path / f"face_{i}.stl"
                export_stl(face, fname)

                try:
                    # Load the face mesh
                    face_mesh = trimesh.load(fname)
                    if isinstance(face_mesh, trimesh.Scene):
                        face_mesh = face_mesh.dump(concatenate=True)

                    # Add to scene with specific name for UI selection
                    # The node name allows the frontend to identify it as 'face_X'
                    scene.add_geometry(
                        face_mesh,
                        node_name=f"face_{i}",
                        geom_name=f"face_{i}_geom",
                    )
                except Exception as e:
                    logger.warning(f"failed_to_export_face_{i}", error=str(e))

        # Export the scene to GLB
        scene.export(filepath, file_type="glb")

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

        # Body lookup cache
        self.body_elements: dict[str, ET.Element] = {}

        # Equality constraints
        self.equality = ET.SubElement(self.root, "equality")

    def add_mesh_asset(self, name: str, file_name: str):
        """Registers a mesh file in the MJCF assets."""
        ET.SubElement(self.assets, "mesh", name=name, file=file_name)

    def add_site(
        self,
        name: str,
        pos: list[float],
        size: float = 0.001,
        rgba: str | None = None,
        parent_body_name: str | None = None,
    ):
        """Adds a site to the worldbody or a specific body."""
        parent = self.worldbody
        if parent_body_name and parent_body_name in self.body_elements:
            parent = self.body_elements[parent_body_name]

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
        tendon_range: list[float] | None = None,
        stiffness: float | None = None,
        damping: float | None = None,
    ):
        """Adds a spatial tendon connecting a sequence of sites."""
        if not hasattr(self, "tendon_element"):
            self.tendon_element = ET.SubElement(self.root, "tendon")

        attrs = {"name": name, "width": str(width), "rgba": rgba}
        if limited and tendon_range:
            attrs["limited"] = "true"
            attrs["range"] = " ".join(map(str, tendon_range))
        if stiffness is not None:
            attrs["stiffness"] = str(stiffness)
        if damping is not None:
            attrs["damping"] = str(damping)

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
        self.body_elements[name] = body
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
        cots_id: str | None = None,
    ):
        """Adds an actuator (motor/servo) to control a joint."""
        # Default gains if not derived
        final_kp = kp if kp is not None else 10.0
        final_kv = kv if kv is not None else 1.0
        final_forcerange = forcerange

        # Try to derive from COTS if missing parameters
        if not cots_id:
            # Heuristic: find known motor ID in name
            from shared.cots.parts.motors import ServoMotor

            for motor_id in ServoMotor.motor_data:
                if motor_id in name:
                    cots_id = motor_id
                    break

        if cots_id:
            from shared.cots.parts.motors import retrieve_cots_physics

            physics = retrieve_cots_physics(cots_id)
            if physics:
                if forcerange is None:
                    torque = physics["torque"]
                    final_forcerange = (-torque, torque)
                if kp is None:
                    final_kp = physics["kp"]
                    if kv is None:
                        final_kv = physics["kv"]

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
        with path.open("w") as f:
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
        parts_data = CommonAssemblyTraverser.traverse(assembly, electronics)
        cots_lookup = {d.label: d.cots_id for d in parts_data}

        for data in parts_data:
            if data.constraint and data.constraint.startswith("weld:"):
                target = data.constraint.split(":")[1]
                weld_constraints.append((data.label, target))

        for data in parts_data:
            if data.is_zone:
                self.compiler.add_body(
                    name=data.label,
                    is_zone=True,
                    zone_type=data.zone_type,
                    zone_size=data.zone_size,
                    pos=data.pos,
                    euler=data.euler,
                )
            else:
                mesh_path_base = self.assets_dir / data.label
                saved_paths = self.processor.process_geometry(
                    data.part, mesh_path_base, use_vhacd=self.use_vhacd
                )
                obj_paths = [p for p in saved_paths if p.suffix == ".obj"]

                mesh_names = []
                for j, path in enumerate(obj_paths):
                    mesh_name = (
                        f"{data.label}_{j}" if len(obj_paths) > 1 else data.label
                    )
                    self.compiler.add_mesh_asset(name=mesh_name, file_name=path.name)
                    mesh_names.append(mesh_name)

                self.compiler.add_body(
                    name=data.label,
                    mesh_names=mesh_names,
                    pos=data.pos,
                    euler=data.euler,
                    is_fixed=data.is_fixed,
                    joint_type=data.joint_type,
                    joint_axis=data.joint_axis,
                    joint_range=data.joint_range,
                )
                body_locations[data.label] = (data.pos, data.euler)

        # Apply collected constraints (welds are handled as joints in traverse for now,
        # but manual welds metadata can be added to traverse if needed)
        # TODO: Unified weld resolution in traverse
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
                        cots_id=cots_lookup.get(mp.part_name),
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
                        parent = None
                        local_pos = list(pt)

                        # Heuristic: first waypoint attached to from_comp, last to to_comp
                        if j == 0:
                            comp_id = wire.from_terminal.component
                            if comp_id in body_locations:
                                parent = comp_id
                                b_pos, _ = body_locations[comp_id]
                                local_pos = [pt[k] - b_pos[k] for k in range(3)]
                        elif j == len(waypoints) - 1:
                            comp_id = wire.to_terminal.component
                            if comp_id in body_locations:
                                parent = comp_id
                                b_pos, _ = body_locations[comp_id]
                                local_pos = [pt[k] - b_pos[k] for k in range(3)]

                        self.compiler.add_site(
                            name=site_name, pos=local_pos, parent_body_name=parent
                        )
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
                    # Wire material properties:
                    # Copper has high stiffness, but tendons are 1D.
                    # We use a value that prevents too much stretching.
                    stiffness = 1000.0 * (1.26 ** (18 - wire.gauge_awg))
                    self.compiler.add_spatial_tendon(
                        name=wire.wire_id,
                        site_names=site_names,
                        width=max(0.0005, width),
                        stiffness=stiffness,
                        damping=stiffness * 0.1,
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

        scene_data = {
            "entities": [],
            "fluids": [],
            "objectives": [],
            "motors": [],
            "cables": [],
        }

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
        parts_data = CommonAssemblyTraverser.traverse(assembly, electronics)

        # Load manufacturing config to check for deformable materials
        from worker.workbenches.config import load_config

        mfg_config = load_config()

        for data in parts_data:
            # Check if deformable
            is_deformable = False
            if objectives and objectives.physics and objectives.physics.fem_enabled:
                is_deformable = True
            else:
                mat_def = mfg_config.materials.get(data.material_id)
                if mat_def and mat_def.material_class in ["soft", "elastomer"]:
                    is_deformable = True

            mesh_path_base = self.assets_dir / data.label
            self.processor.process_geometry(data.part, mesh_path_base, decompose=False)

            entity_info = {
                "name": data.label,
                "pos": data.pos,
                "euler": data.euler,
                "material_id": data.material_id,
                "fixed": data.is_fixed,
                "joint": {
                    "type": data.joint_type,
                    "axis": data.joint_axis,
                    "range": data.joint_range,
                }
                if data.joint_type
                else None,
            }

            # WP3 Forward Compatibility: Mark as electronics if referenced
            if data.is_electronics:
                entity_info["is_electronics"] = True

            if is_deformable:
                msh_path = mesh_path_base.with_suffix(".msh")
                stl_path = mesh_path_base.with_suffix(".stl")
                repaired_stl_path = mesh_path_base.with_suffix(".repaired.stl")

                # Export to STL for processing
                export_stl(data.part, str(stl_path))

                from worker.utils.mesh_utils import repair_mesh_file, tetrahedralize

                try:
                    # T008: Repair mesh before tetrahedralization
                    repair_mesh_file(stl_path, repaired_stl_path)
                    # T007: Tetrahedralize the repaired mesh
                    tetrahedralize(repaired_stl_path, msh_path)
                finally:
                    # Cleanup intermediate files
                    if stl_path.exists():
                        stl_path.unlink()
                    if repaired_stl_path.exists():
                        repaired_stl_path.unlink()

                entity_info["type"] = "soft_mesh"
                entity_info["file"] = str(msh_path.relative_to(self.assets_dir.parent))
            else:
                entity_info["type"] = "mesh"
                entity_info["file"] = str(
                    mesh_path_base.with_suffix(".obj").relative_to(
                        self.assets_dir.parent
                    )
                )

            scene_data["entities"].append(entity_info)

        # 3. Add moving parts (Motors)
        if moving_parts:
            for mp in moving_parts:
                scene_data["motors"].append(mp.model_dump())

        # 4. Add Electronics (Cables)
        if electronics and hasattr(electronics, "wiring"):
            for wire in electronics.wiring:
                if getattr(wire, "routed_in_3d", False):
                    # In Genesis, we can represent cables with their waypoints
                    # and radius derived from AWG.
                    # radius scaling: AWG 10 is ~1.25mm, AWG 20 is ~0.4mm
                    radius = 0.0005 + (20 - wire.gauge_awg) * 0.00005
                    # Stiffness scaling
                    stiffness = 1000.0 * (1.26 ** (18 - wire.gauge_awg))

                    scene_data["cables"].append(
                        {
                            "wire_id": wire.wire_id,
                            "gauge_awg": wire.gauge_awg,
                            "radius": max(0.00025, radius),
                            "stiffness": stiffness,
                            "path": [list(pt) for pt in wire.waypoints],
                        }
                    )

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
        with scene_path.open("w") as f:
            json.dump(scene_data, f, indent=2)

        return scene_path
