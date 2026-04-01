from __future__ import annotations

from typing import Any


def export_preview_scene_bundle(*args: Any, **kwargs: Any) -> str:
    """Export a preview scene bundle through the current preview implementation."""
    from worker_heavy.utils.build123d_rendering import (
        export_preview_scene_bundle as _export_preview_scene_bundle,
    )

    return _export_preview_scene_bundle(*args, **kwargs)
