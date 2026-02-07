import mujoco
import numpy as np
import ffmpeg
import os
from pathlib import Path
from PIL import Image
import zipfile
import structlog

logger = structlog.get_logger(__name__)

class Renderer:
    """Handles offscreen rendering and artifact generation for MuJoCo simulations."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, width: int = 640, height: int = 480):
        self.model = model
        self.data = data
        self.width = width
        self.height = height
        # Offscreen rendering initialization
        self.renderer = mujoco.Renderer(model, height, width)
        self.frames = []

    def render_frame(self, camera: int = -1) -> np.ndarray:
        """Renders a single frame from the specified camera."""
        self.renderer.update_scene(self.data, camera=camera)
        return self.renderer.render()

    def render_24_views(self) -> list[np.ndarray]:
        """
        Renders 24 views of the current scene (8 horizontal x 3 vertical).
        Uses the free camera and orbits it around the lookat point.
        """
        views = []
        # Create a camera object
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        
        # Center of the scene
        cam.lookat = np.array([0, 0, 0.5])
        cam.distance = 2.0
        
        elevations = [-15, -45, -75]  # MuJoCo elevation is usually negative for "top-down"
        azimuths = [0, 45, 90, 135, 180, 225, 270, 315]
        
        for elev in elevations:
            for azim in azimuths:
                cam.elevation = elev
                cam.azimuth = azim
                self.renderer.update_scene(self.data, camera=cam)
                views.append(self.renderer.render())
        
        return views

    def save_artifacts(self, output_dir: Path, fps: int = 30):
        """
        Encodes captured frames to MP4 and bundles 24 views into a zip.
        """
        output_dir.mkdir(parents=True, exist_ok=True)
        renders_dir = output_dir / "renders"
        renders_dir.mkdir(parents=True, exist_ok=True)

        # 1. Save Video
        video_path = renders_dir / "simulation.mp4"
        if self.frames:
            self._encode_video(video_path, fps)
            logger.info("video_saved", path=str(video_path), frames=len(self.frames))
        else:
            logger.warning("no_frames_to_save")

        # 2. Save 24-View Bundle
        views = self.render_24_views()
        bundle_path = renders_dir / "preview_bundle.zip"
        self._create_preview_bundle(bundle_path, views)
        logger.info("preview_bundle_saved", path=str(bundle_path))

        return video_path, bundle_path

    def _encode_video(self, path: Path, fps: int):
        """Uses ffmpeg-python to encode frames to MP4."""
        if not self.frames:
            return

        # Ensure directory exists
        path.parent.mkdir(parents=True, exist_ok=True)

        height, width, _ = self.frames[0].shape
        
        process = (
            ffmpeg
            .input('pipe:', format='rawvideo', pix_fmt='rgb24', s=f'{width}x{height}', r=fps)
            .output(str(path), vcodec='libx264', pix_fmt='yuv420p')
            .overwrite_output()
            .run_async(pipe_stdin=True, pipe_stderr=True)
        )

        try:
            for frame in self.frames:
                process.stdin.write(frame.tobytes())
            
            process.stdin.close()
            process.wait()
        except BrokenPipeError:
            _, stderr = process.communicate()
            logger.error("ffmpeg_failed", stderr=stderr.decode() if stderr else "No stderr")
            raise



    def _create_preview_bundle(self, path: Path, views: list[np.ndarray]):
        """Saves views as JPEGs and zips them."""
        with zipfile.ZipFile(path, 'w') as zipf:
            for i, view in enumerate(views):
                img = Image.fromarray(view)
                img_name = f"view_{i:02d}.jpg"
                img_temp = Path(f"/tmp/{img_name}")
                img.save(img_temp, "JPEG")
                zipf.write(img_temp, img_name)
                img_temp.unlink()

    def add_frame(self):
        """Captures the current frame and adds it to the frame list."""
        self.frames.append(self.render_frame())
