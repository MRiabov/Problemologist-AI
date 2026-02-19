from pathlib import Path
from typing import Any

from build123d import Solid

from shared.workers.loader import load_component_from_script

# Alias for internal use in rendering activity
_load_component = load_component_from_script


def inspect_topology(
    target_id: str, script_path: str | Path = "script.py"
) -> dict[str, Any]:
    """
    Inspects a specific topological feature and returns its properties.
    target_id format: 'face_12', 'edge_5', 'part_0', etc.
    """
    component = load_component_from_script(Path(script_path))

    # parts = component.solids()
    # if not parts:
    #     parts = component.children

    # Handle 'part_N'
    if target_id.startswith("part_"):
        try:
            idx = int(target_id.split("_")[1])
            # Solids in the compound
            solids = component.solids()
            if not solids:
                solids = [component] if isinstance(component, Solid) else []

            part = solids[idx]
            bbox = part.bounding_box()
            return {
                "type": "part",
                "index": idx,
                "center": (part.center().X, part.center().Y, part.center().Z),
                "bbox": {
                    "min": (bbox.min.X, bbox.min.Y, bbox.min.Z),
                    "max": (bbox.max.X, bbox.max.Y, bbox.max.Z),
                },
            }
        except (ValueError, IndexError):
            raise ValueError(f"Invalid part ID: {target_id}")

    # Handle 'face_N'
    if target_id.startswith("face_"):
        try:
            idx = int(target_id.split("_")[1])
            all_faces = component.faces()
            face = all_faces[idx]
            bbox = face.bounding_box()
            # normal_at defaults to center if no point is provided
            normal = face.normal_at()
            return {
                "type": "face",
                "index": idx,
                "center": (face.center().X, face.center().Y, face.center().Z),
                "normal": (normal.X, normal.Y, normal.Z),
                "area": face.area,
                "bbox": {
                    "min": (bbox.min.X, bbox.min.Y, bbox.min.Z),
                    "max": (bbox.max.X, bbox.max.Y, bbox.max.Z),
                },
            }
        except (ValueError, IndexError):
            raise ValueError(f"Invalid face ID: {target_id}")

    # Handle 'edge_N'
    if target_id.startswith("edge_"):
        try:
            idx = int(target_id.split("_")[1])
            all_edges = component.edges()
            edge = all_edges[idx]
            bbox = edge.bounding_box()
            return {
                "type": "edge",
                "index": idx,
                "center": (edge.center().X, edge.center().Y, edge.center().Z),
                "length": edge.length,
                "bbox": {
                    "min": (bbox.min.X, bbox.min.Y, bbox.min.Z),
                    "max": (bbox.max.X, bbox.max.Y, bbox.max.Z),
                },
            }
        except (ValueError, IndexError):
            raise ValueError(f"Invalid edge ID: {target_id}")

    raise ValueError(f"Unknown target ID format: {target_id}")
