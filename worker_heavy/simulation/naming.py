"""Shared naming helpers for simulation scene identifiers."""

MOVED_OBJECT_SCENE_PREFIX = "benchmark_moved_object__"


def moved_object_scene_name(label: str) -> str:
    """Return a namespaced scene identifier for the benchmark payload."""
    clean_label = str(label).strip()
    if not clean_label:
        raise ValueError("payload label must be non-empty")
    return f"{MOVED_OBJECT_SCENE_PREFIX}{clean_label}"


def is_moved_object_scene_name(name: str) -> bool:
    """Return True when a scene name belongs to the benchmark payload."""
    return str(name).startswith(MOVED_OBJECT_SCENE_PREFIX)
