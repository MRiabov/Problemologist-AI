from pathlib import Path

import structlog

from shared.simulation.backends import RendererBackend
from worker_heavy.utils.rendering import VideoRenderer

logger = structlog.get_logger(__name__)


class MediaRecorder:
    """Handles video recording for simulations."""

    def __init__(
        self,
        video_path: Path | None,
        capture_interval: int = 15,
        session_id: str | None = None,
    ):
        self.video_renderer = (
            VideoRenderer(video_path, session_id=session_id) if video_path else None
        )
        self.capture_interval = capture_interval
        self.session_id = session_id

    def update(self, step_idx: int, backend: RendererBackend):
        """Capture a frame if at the right interval."""
        if not self.video_renderer or step_idx % self.capture_interval != 0:
            return
        if step_idx == 0 and self.video_renderer.frames:
            return

        try:
            # Setup a reasonable camera for video if not already set
            # For MuJoCo we can use "free" or a named camera.
            # For Genesis we use "main".
            cameras = backend.get_all_camera_names()
            camera_candidates = ["main"]
            if "main" in cameras:
                camera_candidates.extend([name for name in cameras if name != "main"])
            else:
                camera_candidates.extend(cameras)

            frame = None
            last_error: Exception | None = None
            for cam_to_use in camera_candidates:
                try:
                    frame = backend.render_camera(cam_to_use, 640, 480)
                    break
                except Exception as exc:
                    last_error = exc
                    if "Unknown MuJoCo camera" not in str(exc):
                        raise

            if frame is None:
                if last_error is not None:
                    raise last_error
                return

            particles = backend.get_particle_positions()
            self.video_renderer.add_frame(frame, particles=particles)
        except Exception as e:
            # T024: Skip rendering if display is not available (e.g. CI without GPU)
            if "EGL" in str(e) or "display" in str(e).lower():
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                self.video_renderer = None  # Stop attempting to render video
            elif "Unknown MuJoCo camera" in str(e):
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                self.video_renderer = None
            else:
                raise

    def save(self):
        """Finalize and save the video."""
        if self.video_renderer:
            self.video_renderer.save()
