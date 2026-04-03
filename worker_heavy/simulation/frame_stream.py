from __future__ import annotations

import base64
import io
from urllib.parse import urlparse, urlunparse

import numpy as np
import structlog
from PIL import Image

from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import SimulationFrameStreamMessage

logger = structlog.get_logger(__name__)


def _http_to_ws_url(base_url: str, path: str) -> str:
    parsed = urlparse(base_url.rstrip("/"))
    scheme = "wss" if parsed.scheme == "https" else "ws"
    return urlunparse(
        (
            scheme,
            parsed.netloc,
            parsed.path.rstrip("/") + path,
            parsed.params,
            parsed.query,
            parsed.fragment,
        )
    )


def _encode_png_base64(frame: np.ndarray) -> str:
    buffer = io.BytesIO()
    Image.fromarray(frame).save(buffer, format="PNG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class SimulationFrameStreamPublisher:
    """Best-effort live frame publisher for simulation evidence."""

    def __init__(
        self,
        *,
        controller_url: str | None,
        episode_id: str | None,
        session_id: str | None,
        enabled: bool,
    ) -> None:
        self.controller_url = controller_url
        self.episode_id = episode_id
        self.session_id = session_id or "simulation"
        self.enabled = bool(enabled and controller_url and episode_id)
        self._connection = None
        self._disabled_reason: str | None = None

    @property
    def is_enabled(self) -> bool:
        return self.enabled and self._disabled_reason is None

    def _stream_url(self) -> str:
        if not self.controller_url or not self.episode_id:
            raise RuntimeError("simulation frame stream missing controller target")
        return _http_to_ws_url(
            self.controller_url,
            f"/api/episodes/{self.episode_id}/simulation-stream/ws",
        )

    def _ensure_connection(self) -> None:
        if not self.is_enabled:
            return
        if self._connection is not None:
            return
        from websockets.sync.client import connect as websocket_connect

        headers = {"X-Session-ID": self.session_id}
        self._connection = websocket_connect(
            self._stream_url(),
            additional_headers=headers,
            open_timeout=10.0,
            close_timeout=5.0,
            ping_interval=20.0,
            ping_timeout=20.0,
            max_size=None,
        )

    def publish_frame(
        self,
        *,
        frame: np.ndarray,
        frame_index: int,
        simulation_time_s: float,
        capture_interval_seconds: float,
        backend_type: SimulatorBackendType,
        backend_name: str,
        resolved_camera_name: str | None,
        used_default_view: bool,
        resolved_default_view_label: str | None,
    ) -> None:
        if not self.is_enabled:
            return

        try:
            self._ensure_connection()
            if self._connection is None:
                return
            message = SimulationFrameStreamMessage(
                episode_id=self.episode_id or "",
                session_id=self.session_id,
                frame_index=frame_index,
                simulation_time_s=simulation_time_s,
                capture_interval_seconds=capture_interval_seconds,
                backend_type=backend_type,
                backend_name=backend_name,
                resolved_camera_name=resolved_camera_name,
                used_default_view=used_default_view,
                resolved_default_view_label=resolved_default_view_label,
                image_bytes_base64=_encode_png_base64(frame),
            )
            self._connection.send(message.model_dump_json())
        except Exception as exc:
            logger.warning(
                "simulation_frame_stream_disabled",
                error=str(exc),
                episode_id=self.episode_id,
                session_id=self.session_id,
            )
            self._disabled_reason = str(exc)
            self.close()

    def close(self) -> None:
        if self._connection is None:
            return
        try:
            self._connection.close()
        finally:
            self._connection = None
