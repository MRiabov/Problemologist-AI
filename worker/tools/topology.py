import logging
import importlib.util
import sys
from pathlib import Path
from typing import Any
from build123d import Compound, Solid

logger = logging.getLogger(__name__)


def _load_component(script_path: str = "script.py") -> Compound:
    """Loads the component from the specified script."""
    path = Path(script_path)
    if not path.exists():
        raise FileNotFoundError(f"Script not found at {path.absolute()}")

    # Add directory to sys.path
    if str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))

    spec = importlib.util.spec_from_file_location("dynamic_build", str(path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load spec for {path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    if hasattr(module, "build"):
        return module.build()

    # Try finding build in local scope if it's a script
    for attr in dir(module):
        val = getattr(module, attr)
        if callable(val) and attr == "build":
            return val()

    raise AttributeError("build() function not found in script.")


def inspect_topology(target_id: str, script_path: str = "script.py") -> dict[str, Any]:
    """
    Inspects a specific topological feature and returns its properties.
    target_id format: 'face_12', 'edge_5', 'part_0', etc.
    """
    component = _load_component(script_path)

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
