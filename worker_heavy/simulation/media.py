import json
from pathlib import Path

import numpy as np
import structlog

from shared.agents import get_video_render_resolution
from shared.models.simulation import (
    RendererCapabilities,
    SimulationRenderProvenance,
)
from shared.rendering import render_simulation_video_artifact
from shared.simulation.backends import RendererBackend
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import RenderFrameMetadata
from worker_heavy.simulation.frame_stream import SimulationFrameStreamPublisher

logger = structlog.get_logger(__name__)


class MediaRecorder:
    """Handles video recording for simulations."""

    def __init__(
        self,
        video_path: Path | None,
        *,
        backend_type: SimulatorBackendType,
        capture_interval_seconds: float = 0.5,
        session_id: str | None = None,
        render_resolution: tuple[int, int] | None = None,
        frame_stream_publisher: SimulationFrameStreamPublisher | None = None,
    ):
        self.video_path = video_path
        self.frames: list[np.ndarray] = []
        self.captured_sim_times: list[float] = []
        self.capture_interval_seconds = capture_interval_seconds
        self.session_id = session_id
        self.backend_type = backend_type
        self.frame_stream_publisher = frame_stream_publisher
        if render_resolution is None:
            render_resolution = get_video_render_resolution()
        self.render_width, self.render_height = render_resolution
        self.render_provenance: SimulationRenderProvenance | None = None
        self._capture_disabled = False
        self._next_capture_time_s = 0.0

    def _ensure_render_provenance(
        self, backend: RendererBackend
    ) -> tuple[SimulationRenderProvenance, RendererCapabilities]:
        capabilities = backend.get_render_capabilities()
        if self.render_provenance is None:
            self.render_provenance = SimulationRenderProvenance(
                backend_type=self.backend_type,
                backend_name=capabilities.backend_name,
                renderer_capabilities=capabilities,
                capture_interval_seconds=self.capture_interval_seconds,
            )
        else:
            self.render_provenance.backend_name = capabilities.backend_name
            self.render_provenance.renderer_capabilities = capabilities
            if self.render_provenance.capture_interval_seconds is None:
                self.render_provenance.capture_interval_seconds = (
                    self.capture_interval_seconds
                )
        return self.render_provenance, capabilities

    def _capture_frame(
        self, backend: RendererBackend
    ) -> tuple[np.ndarray, SimulationRenderProvenance]:
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
                frame = backend.render_camera(
                    cam_to_use, self.render_width, self.render_height
                )
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

        return frame, render_provenance

    def update(self, sim_time_s: float, backend: RendererBackend):
        """Capture a frame at the configured simulation-time cadence."""
        if self._capture_disabled:
            return
        if self.video_path is None and self.frame_stream_publisher is None:
            return
        if sim_time_s + 1e-9 < self._next_capture_time_s:
            return

        try:
            frame, render_provenance = self._capture_frame(backend)
            self.render_provenance = render_provenance
            self.frames.append(frame)
            self.captured_sim_times.append(sim_time_s)
            render_provenance.captured_frame_count += 1
            self._next_capture_time_s = sim_time_s + self.capture_interval_seconds

            if self.frame_stream_publisher is not None:
                self.frame_stream_publisher.publish_frame(
                    frame=frame,
                    frame_index=len(self.frames) - 1,
                    simulation_time_s=sim_time_s,
                    capture_interval_seconds=self.capture_interval_seconds,
                    backend_type=self.backend_type,
                    backend_name=render_provenance.backend_name,
                    resolved_camera_name=render_provenance.resolved_camera_name,
                    used_default_view=render_provenance.used_default_view,
                    resolved_default_view_label=(
                        render_provenance.resolved_default_view_label
                    ),
                )
        except Exception as e:
            # T024: Skip rendering if display is not available (e.g. CI without GPU)
            if "EGL" in str(e) or "display" in str(e).lower():
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                self._capture_disabled = True
            elif "Unknown MuJoCo camera" in str(e):
                logger.warning("camera_render_failed_skipping_video", error=str(e))
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                self._capture_disabled = True
            else:
                if self.render_provenance is not None:
                    self.render_provenance.render_error = str(e)
                raise

    def save(self) -> str | None:
        """Finalize and save the video."""
        if self.video_path is not None and self.frames:
            fps = max(1, int(round(1.0 / self.capture_interval_seconds)))
            rendered = render_simulation_video_artifact(
                self.frames,
                output_name=self.video_path.name,
                fps=fps,
                session_id=self.session_id or "simulation",
            )
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            self.video_path.write_bytes(rendered.video_bytes)
            frames_sidecar = self.video_path.parent / "frames.jsonl"
            frame_rows = [
                RenderFrameMetadata(
                    frame_index=index,
                    source_path=str(self.video_path.name),
                    timestamp_s=self.captured_sim_times[index]
                    if index < len(self.captured_sim_times)
                    else index / float(fps),
                ).model_dump(mode="json")
                for index in range(len(self.frames))
            ]
            frames_sidecar.write_text(
                "\n".join(json.dumps(row, sort_keys=False) for row in frame_rows)
                + ("\n" if frame_rows else ""),
                encoding="utf-8",
            )
            logger.info(
                "video_render_complete",
                path=str(self.video_path),
                session_id=self.session_id,
                object_store_key=rendered.object_store_key,
            )
            return rendered.object_store_key
        return None
