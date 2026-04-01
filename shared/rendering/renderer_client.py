from __future__ import annotations

import base64
import os
import tempfile
import time
from pathlib import Path
from typing import Any

import httpx
import structlog

from shared.workers.bundling import bundle_directory_base64
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    PreviewDesignRequest,
    PreviewDesignResponse,
    PreviewRenderingType,
    SimulationVideoRequest,
)

logger = structlog.get_logger(__name__)


def renderer_base_url() -> str:
    return os.getenv("WORKER_RENDERER_URL", "http://worker-renderer:8003").rstrip("/")


def bundle_workspace_base64(root: Path) -> str:
    return bundle_directory_base64(root)


def _session_headers(session_id: str | None) -> dict[str, str]:
    headers: dict[str, str] = {}
    if session_id:
        headers["x-session-id"] = session_id
    return headers


def _write_text_atomic(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=str(path.parent), delete=False
    ) as tmp:
        tmp.write(content)
        tmp_path = Path(tmp.name)
    tmp_path.replace(path)


def _post_json_with_busy_retry(
    *,
    url: str,
    payload: dict[str, Any],
    session_id: str | None,
    timeout: float,
    attempts: int | None = None,
    initial_delay_s: float = 0.5,
    max_delay_s: float = 5.0,
) -> dict[str, Any]:
    """POST JSON to the renderer, retrying transient worker-busy responses."""

    delay_s = initial_delay_s
    busy_retry_deadline_s = time.monotonic() + timeout
    attempt = 0

    while True:
        attempt += 1
        with httpx.Client(
            timeout=timeout, headers=_session_headers(session_id)
        ) as client:
            response = client.post(url, json=payload)
            try:
                response.raise_for_status()
                return response.json()
            except httpx.HTTPStatusError:
                if response.status_code != 503:
                    raise
                if attempts is not None and attempt >= attempts:
                    raise
                if time.monotonic() >= busy_retry_deadline_s:
                    raise

                logger.warning(
                    "renderer_busy_retry",
                    url=url,
                    attempt=attempt,
                    attempts=attempts,
                    delay_s=delay_s,
                    session_id=session_id,
                )
                remaining_s = busy_retry_deadline_s - time.monotonic()
                if remaining_s <= 0:
                    raise
                time.sleep(min(delay_s, remaining_s))
                delay_s = min(delay_s * 1.5, max_delay_s)


def render_preview(
    *,
    bundle_base64: str | None,
    script_path: str,
    orbit_pitch: float,
    orbit_yaw: float,
    rendering_type: PreviewRenderingType = PreviewRenderingType.RGB,
    session_id: str | None = None,
    script_content: str | None = None,
    smoke_test_mode: bool | None = None,
) -> PreviewDesignResponse:
    payload = PreviewDesignRequest(
        bundle_base64=bundle_base64,
        script_path=script_path,
        orbit_pitch=orbit_pitch,
        orbit_yaw=orbit_yaw,
        rendering_type=rendering_type,
        script_content=script_content,
        smoke_test_mode=smoke_test_mode,
    ).model_dump(mode="json")
    url = f"{renderer_base_url()}/benchmark/preview"
    data = _post_json_with_busy_retry(
        url=url,
        payload=payload,
        session_id=session_id,
        timeout=60.0,
    )
    return PreviewDesignResponse.model_validate(data)


def render_static_preview(
    *,
    bundle_base64: str | None,
    script_path: str,
    session_id: str,
    smoke_test_mode: bool | None = None,
    particle_budget: int | None = None,
    script_content: str | None = None,
) -> BenchmarkToolResponse:
    payload = BenchmarkToolRequest(
        bundle_base64=bundle_base64,
        script_path=script_path,
        smoke_test_mode=smoke_test_mode,
        particle_budget=particle_budget,
        script_content=script_content,
    ).model_dump(mode="json")
    payload["session_id"] = session_id
    url = f"{renderer_base_url()}/benchmark/static-preview"
    data = _post_json_with_busy_retry(
        url=url,
        payload=payload,
        session_id=session_id,
        timeout=120.0,
    )
    return BenchmarkToolResponse.model_validate(data)


def render_simulation_video(
    *,
    bundle_base64: str | None,
    frame_paths: list[str],
    output_name: str,
    fps: int,
    session_id: str,
) -> BenchmarkToolResponse:
    payload = SimulationVideoRequest(
        bundle_base64=bundle_base64,
        frame_paths=frame_paths,
        output_name=output_name,
        fps=fps,
        session_id=session_id,
    ).model_dump(mode="json")
    url = f"{renderer_base_url()}/benchmark/simulation-video"
    data = _post_json_with_busy_retry(
        url=url,
        payload=payload,
        session_id=session_id,
        timeout=120.0,
    )
    return BenchmarkToolResponse.model_validate(data)


def materialize_preview_response(
    response: PreviewDesignResponse, output_dir: Path
) -> Path | None:
    """Persist a preview response payload into the local workspace."""
    output_dir.mkdir(parents=True, exist_ok=True)
    if response.image_bytes_base64:
        image_name = Path(response.image_path or "preview.jpg").name
        image_path = output_dir / image_name
        image_path.write_bytes(base64.b64decode(response.image_bytes_base64))
        if response.render_manifest_json:
            manifest_path = output_dir.parent / "render_manifest.json"
            _write_text_atomic(manifest_path, response.render_manifest_json)
        return image_path
    if response.image_path:
        if response.render_manifest_json:
            manifest_path = output_dir.parent / "render_manifest.json"
            _write_text_atomic(manifest_path, response.render_manifest_json)
        return output_dir / Path(response.image_path).name
    if response.render_manifest_json:
        manifest_path = output_dir.parent / "render_manifest.json"
        _write_text_atomic(manifest_path, response.render_manifest_json)
    return None


def materialize_render_artifacts(
    artifacts: Any, workspace_root: Path, *, default_subdir: str = "renders"
) -> list[str]:
    """Persist renderer artifact blobs into the caller's workspace."""
    render_paths: list[str] = []
    render_blobs = getattr(artifacts, "render_blobs_base64", None) or {}
    for rel_path, blob in render_blobs.items():
        target = workspace_root / rel_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(base64.b64decode(blob))
        render_paths.append(str(Path(rel_path)))
    if not render_paths:
        render_paths = [
            str(Path(path)) for path in getattr(artifacts, "render_paths", [])
        ]
    return render_paths
