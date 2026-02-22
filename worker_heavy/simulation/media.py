import structlog
from pathlib import Path
from worker_heavy.utils.rendering import VideoRenderer

logger = structlog.get_logger(__name__)


class MediaRecorder:
    """Handles video recording for simulations."""

    def __init__(self, video_path: Path | None, capture_interval: int = 15):
        self.video_renderer = VideoRenderer(video_path) if video_path else None
        self.capture_interval = capture_interval

    def update(self, step_idx, backend):
        """Capture a frame if at the right interval."""
        if not self.video_renderer or step_idx % self.capture_interval != 0:
            return

        try:
            # Setup a reasonable camera for video if not already set
            # For MuJoCo we can use "free" or a named camera.
            # For Genesis we use "main".
            cameras = backend.get_all_camera_names()
            cam_to_use = "main"
            if "main" not in cameras and cameras:
                cam_to_use = cameras[0]

            frame = backend.render_camera(cam_to_use, 640, 480)
            particles = backend.get_particle_positions()
            self.video_renderer.add_frame(frame, particles=particles)
        except Exception as e:
            # T024: Skip rendering if display is not available (e.g. CI without GPU)
            if "EGL" in str(e) or "display" in str(e).lower():
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                self.video_renderer = None  # Stop attempting to render video
            else:
                raise

    def save(self):
        """Finalize and save the video."""
        if self.video_renderer:
            self.video_renderer.save()
