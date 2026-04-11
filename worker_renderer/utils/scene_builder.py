from __future__ import annotations

from shared.simulation.scene_builder import (
    MOVED_OBJECT_SCENE_PREFIX,
    AssemblyPartData,
    CommonAssemblyTraverser,
    MaterializedMovedObject,
    MeshProcessor,
    PreviewEntity,
    PreviewScene,
    build_moved_object_geometry,
    build_moved_object_start_geometry,
    is_moved_object_scene_name,
    materialize_moved_object,
    moved_object_scene_name,
    normalize_preview_label,
)

__all__ = [
    "AssemblyPartData",
    "CommonAssemblyTraverser",
    "MaterializedMovedObject",
    "MeshProcessor",
    "MOVED_OBJECT_SCENE_PREFIX",
    "PreviewEntity",
    "PreviewScene",
    "build_moved_object_geometry",
    "build_moved_object_start_geometry",
    "is_moved_object_scene_name",
    "materialize_moved_object",
    "moved_object_scene_name",
    "normalize_preview_label",
]
