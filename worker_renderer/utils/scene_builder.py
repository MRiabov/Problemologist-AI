from __future__ import annotations

import re
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import Any

import structlog
import trimesh
from build123d import Compound, Solid, export_stl
from pydantic import BaseModel, ConfigDict

logger = structlog.get_logger(__name__)


def normalize_preview_label(label: Any) -> str:
    """Return a file-safe preview label stem without adding fallback numbering."""
    text = "" if label is None else str(label).strip()
    if not text:
        return ""
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", text)
    cleaned = re.sub(r"_+", "_", cleaned).strip("._-")
    return cleaned


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
    weld_target: str | None = None


class CommonAssemblyTraverser:
    """Unifies assembly traversal and metadata resolution for preview scene export."""

    @staticmethod
    def _resolve_material_id(metadata: Any) -> str | None:
        material_id = getattr(metadata, "material_id", None)
        if material_id:
            return material_id
        if getattr(metadata, "cots_id", None):
            return "cots-generic"
        return None

    @staticmethod
    def traverse(
        assembly: Compound,
        electronics: Any | None = None,
        *,
        allow_unnamed_labels: bool = False,
        unnamed_label_factory: Callable[[], str] | None = None,
    ) -> list[AssemblyPartData]:
        children = getattr(assembly, "children", [])
        if not children:
            children = [assembly]

        parts_data = []
        for i, child in enumerate(children):
            raw_label = getattr(child, "label", None)
            label = normalize_preview_label(raw_label)
            if not label:
                if not allow_unnamed_labels:
                    logger.error(
                        "top_level_build123d_label_missing",
                        part_index=i,
                        raw_label=raw_label,
                        part_type=type(child).__name__,
                    )
                    raise ValueError(
                        "Top-level build123d objects must have non-empty labels. "
                        f"Offending part index: {i}."
                    )
                if unnamed_label_factory is None:
                    label = f"unnamed_{i + 1}"
                else:
                    label = unnamed_label_factory()

            pos, euler = CommonAssemblyTraverser._resolve_location(child)
            meta = CommonAssemblyTraverser._resolve_part_metadata(child)
            zone_info = CommonAssemblyTraverser._detect_zone(child, label)
            is_electronics = CommonAssemblyTraverser._map_electronics(
                label, electronics
            )

            constraint = getattr(child, "constraint", None)
            weld_target = None
            if constraint and constraint.startswith("weld:"):
                weld_target = constraint.split(":")[1]

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
                    constraint=constraint,
                    weld_target=weld_target,
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
            "material_id": CommonAssemblyTraverser._resolve_material_id(metadata),
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
            if "goal" in label:
                zone_type = "goal"
            elif "build" in label:
                zone_type = "build"
            else:
                zone_type = "forbid"
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


class MeshProcessor:
    """Converts build123d geometry into preview-ready mesh files."""

    def process_geometry(
        self,
        part: Solid | Compound,
        filepath: Path,
        decompose: bool = True,
        use_vhacd: bool = False,
        tolerance: float = 0.1,
        angular_tolerance: float = 0.1,
    ) -> list[Path]:
        filepath.parent.mkdir(parents=True, exist_ok=True)
        temp_stl = filepath.with_suffix(".tmp.stl")
        export_stl(
            part, temp_stl, tolerance=tolerance, angular_tolerance=angular_tolerance
        )

        output_paths = []
        try:
            mesh = trimesh.load(temp_stl)
            if isinstance(mesh, trimesh.Scene):
                mesh = mesh.dump(concatenate=True)

            mesh = self._recenter_mesh(mesh)
            mesh = self._validate_watertight(mesh, filepath.name)

            if decompose:
                if use_vhacd:
                    try:
                        decomposed = trimesh.decomposition.convex_decomposition(mesh)
                        if isinstance(decomposed, list) and len(decomposed) > 1:
                            for i, dm in enumerate(decomposed):
                                obj_sub = filepath.with_name(f"{filepath.stem}_{i}.obj")
                                glb_sub = filepath.with_name(f"{filepath.stem}_{i}.glb")
                                dm.export(obj_sub, file_type="obj")
                                dm.export(glb_sub, file_type="glb")
                                output_paths.extend([obj_sub, glb_sub])
                            return output_paths
                    except Exception:
                        pass

                mesh = self.compute_convex_hull(mesh)

            obj_path = filepath.with_suffix(".obj")
            glb_path = filepath.with_suffix(".glb")

            mesh.export(obj_path, file_type="obj")
            try:
                self.export_topology_glb(part, glb_path)
            except Exception as e:
                logger.warning("topology_export_failed", error=str(e))
                mesh.export(glb_path, file_type="glb")

            output_paths.extend([obj_path, glb_path])
        finally:
            if temp_stl.exists():
                temp_stl.unlink()

        return output_paths

    def export_topology_glb(self, part: Solid | Compound, filepath: Path):
        scene = trimesh.Scene()
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            faces = part.faces()
            for i, face in enumerate(faces):
                face_path = tmp_path / f"face_{i}.stl"
                export_stl(face, face_path)
                try:
                    face_mesh = trimesh.load(face_path)
                    if isinstance(face_mesh, trimesh.Scene):
                        face_mesh = face_mesh.dump(concatenate=True)
                    scene.add_geometry(
                        face_mesh,
                        node_name=f"face_{i}",
                        geom_name=f"face_{i}_geom",
                    )
                except Exception as e:
                    logger.warning("failed_to_export_face", face_index=i, error=str(e))
        scene.export(filepath, file_type="glb")

    def _recenter_mesh(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        centroid = mesh.centroid
        mesh.apply_translation(-centroid)
        return mesh

    def _validate_watertight(self, mesh: trimesh.Trimesh, name: str) -> trimesh.Trimesh:
        if mesh.is_watertight:
            return mesh
        try:
            mesh.fill_holes()
            mesh.fix_normals()
            mesh.fix_inversion()
        except Exception:
            pass
        if mesh.is_watertight:
            return mesh
        logger.warning(
            "mesh_not_watertight_falling_back_to_convex_hull",
            mesh_name=name,
        )
        try:
            hull = mesh.convex_hull
            if hull.is_watertight:
                return hull
        except Exception:
            pass
        raise ValueError(
            f"Mesh '{name}' is not watertight. "
            "All meshes must be watertight for physics simulation."
        )

    def compute_convex_hull(self, mesh: trimesh.Trimesh) -> trimesh.Trimesh:
        return mesh.convex_hull
