from __future__ import annotations

from .headless import configure_headless_vtk_egl, create_headless_vtk_render_window
from .preview_scene import export_preview_scene_bundle

__all__ = [
    "bundle_workspace_base64",
    "materialize_preview_response",
    "materialize_render_artifacts",
    "configure_headless_vtk_egl",
    "create_headless_vtk_render_window",
    "render_preview",
    "render_simulation_video_artifact",
    "render_simulation_video",
    "render_simulation_video_bytes",
    "RenderedSimulationVideo",
    "export_preview_scene_bundle",
    "render_static_preview",
    "renderer_base_url",
    "synthesize_placeholder_frames",
]


_LAZY_EXPORTS = {
    "bundle_workspace_base64": ("renderer_client", "bundle_workspace_base64"),
    "materialize_preview_response": ("renderer_client", "materialize_preview_response"),
    "materialize_render_artifacts": ("renderer_client", "materialize_render_artifacts"),
    "render_preview": ("renderer_client", "render_preview"),
    "render_static_preview": ("renderer_client", "render_static_preview"),
    "renderer_base_url": ("renderer_client", "renderer_base_url"),
    "render_simulation_video_artifact": (
        "simulation_video",
        "render_simulation_video_artifact",
    ),
    "render_simulation_video": ("simulation_video", "render_simulation_video_bytes"),
    "render_simulation_video_bytes": (
        "simulation_video",
        "render_simulation_video_bytes",
    ),
    "RenderedSimulationVideo": ("simulation_video", "RenderedSimulationVideo"),
    "synthesize_placeholder_frames": (
        "simulation_video",
        "synthesize_placeholder_frames",
    ),
}


def __getattr__(name: str):
    try:
        module_name, attr_name = _LAZY_EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(name) from exc

    if module_name == "renderer_client":
        from . import renderer_client as module
    else:
        from . import simulation_video as module

    value = getattr(module, attr_name)
    globals()[name] = value
    return value
