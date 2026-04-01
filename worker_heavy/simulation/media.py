from pathlib import Path

import structlog

from shared.models.simulation import (
    RendererCapabilities,
    SimulationRenderProvenance,
)
from shared.simulation.backends import RendererBackend
from shared.simulation.schemas import SimulatorBackendType
from worker_heavy.utils.rendering import VideoRenderer

logger = structlog.get_logger(__name__)


class MediaRecorder:
    """Handles video recording for simulations."""

    def __init__(
        self,
        video_path: Path | None,
        *,
        backend_type: SimulatorBackendType,
        capture_interval: int = 15,
        session_id: str | None = None,
    ):
        self.video_renderer = (
            VideoRenderer(video_path, session_id=session_id) if video_path else None
        )
        self.capture_interval = capture_interval
        self.session_id = session_id
        self.backend_type = backend_type
        self.render_provenance: SimulationRenderProvenance | None = None

    def _ensure_render_provenance(
        self, backend: RendererBackend
    ) -> tuple[SimulationRenderProvenance, RendererCapabilities]:
        capabilities = backend.get_render_capabilities()
        if self.render_provenance is None:
            self.render_provenance = SimulationRenderProvenance(
                backend_type=self.backend_type,
                backend_name=capabilities.backend_name,
                renderer_capabilities=capabilities,
                capture_interval_steps=self.capture_interval,
            )
        else:
            self.render_provenance.backend_name = capabilities.backend_name
            self.render_provenance.renderer_capabilities = capabilities
            if self.render_provenance.capture_interval_steps is None:
                self.render_provenance.capture_interval_steps = self.capture_interval
        return self.render_provenance, capabilities

    def update(self, step_idx: int, backend: RendererBackend):
        """Capture a frame if at the right interval."""
        if not self.video_renderer or step_idx % self.capture_interval != 0:
            return
        if step_idx == 0 and self.video_renderer.frames:
            return

        try:
            render_provenance, capabilities = self._ensure_render_provenance(backend)
            cameras = [
                name
                for name in backend.get_all_camera_names()
                if isinstance(name, str) and name
            ]
            render_provenance.available_camera_names = list(cameras)

            frame = None
            last_error: Exception | None = None
            camera_candidates: list[str] = []
            if cameras and capabilities.supports_named_cameras:
                if "main" in cameras:
                    camera_candidates.append("main")
                camera_candidates.extend(name for name in cameras if name != "main")
            render_provenance.camera_candidates = list(camera_candidates)

            for cam_to_use in camera_candidates:
                try:
                    frame = backend.render_camera(cam_to_use, 640, 480)
                    render_provenance.resolved_camera_name = cam_to_use
                    render_provenance.used_default_view = False
                    render_provenance.resolved_default_view_label = None
                    break
                except Exception as exc:
                    last_error = exc
                    if "Unknown MuJoCo camera" not in str(exc):
                        raise

            if frame is None:
                if not capabilities.supports_default_view:
                    raise RuntimeError(
                        f"{capabilities.backend_name} does not support default-view rendering"
                    )
                try:
                    frame = backend.render()
                    render_provenance.resolved_camera_name = None
                    render_provenance.used_default_view = True
                    render_provenance.resolved_default_view_label = (
                        capabilities.default_view_label
                    )
                    logger.info(
                        "camera_render_fell_back_to_default_view",
                        backend=type(backend).__name__,
                        session_id=self.session_id,
                    )
                except Exception as fallback_error:
                    render_provenance.render_error = str(fallback_error)
                    if last_error is not None:
                        raise last_error from fallback_error
                    raise fallback_error

            particles = backend.get_particle_positions()
            self.video_renderer.add_frame(frame, particles=particles)
            render_provenance.captured_frame_count += 1
        except Exception as e:
            # T024: Skip rendering if display is not available (e.g. CI without GPU)
            if "EGL" in str(e) or "display" in str(e).lower():
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                self.video_renderer = None  # Stop attempting to render video
            elif "Unknown MuJoCo camera" in str(e):
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                self.video_renderer = None
            else:
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                raise

    def save(self) -> str | None:
        """Finalize and save the video."""
        if self.video_renderer:
            return self.video_renderer.save()
        return None
