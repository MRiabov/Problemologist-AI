from __future__ import annotations

import base64
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

from shared.models.simulation import StressFieldData
from shared.observability.storage import S3Client, S3Config
from shared.rendering.renderer_client import (
    bundle_workspace_base64,
    render_stress_heatmap,
)


@dataclass(frozen=True)
class RenderedStressHeatmap:
    """Rendered heatmap payload plus its optional object-store key."""

    image_bytes: bytes
    object_store_key: str | None = None


def _stress_heatmap_s3_client() -> S3Client | None:
    access_key = os.getenv("S3_ACCESS_KEY", os.getenv("AWS_ACCESS_KEY_ID"))
    secret_key = os.getenv("S3_SECRET_KEY", os.getenv("AWS_SECRET_ACCESS_KEY"))
    if not access_key or not secret_key:
        return None

    bucket_name = os.getenv("ASSET_S3_BUCKET", "problemologist")
    return S3Client(
        S3Config(
            endpoint_url=os.getenv("S3_ENDPOINT"),
            access_key_id=access_key,
            secret_access_key=secret_key,
            bucket_name=bucket_name,
            region_name=os.getenv("AWS_REGION", "us-east-1"),
        )
    )


def render_stress_heatmap_artifact(
    stress_field: StressFieldData,
    *,
    output_name: str,
    session_id: str,
    mesh_path: Path | None = None,
    width: int = 800,
    height: int = 600,
) -> RenderedStressHeatmap:
    """Delegate stress-heatmap rendering to worker-renderer and return bytes."""

    with tempfile.TemporaryDirectory() as tmpdir:
        staging_root = Path(tmpdir)
        mesh_rel_path: str | None = None
        if mesh_path is not None and mesh_path.exists():
            staged_mesh = staging_root / mesh_path.name
            staged_mesh.write_bytes(mesh_path.read_bytes())
            mesh_rel_path = staged_mesh.name

        response = render_stress_heatmap(
            bundle_base64=bundle_workspace_base64(staging_root)
            if mesh_rel_path is not None
            else None,
            stress_field=stress_field,
            output_name=output_name,
            session_id=session_id,
            mesh_path=mesh_rel_path,
            width=width,
            height=height,
        )
        if not response.success:
            raise RuntimeError(response.message or "stress heatmap render failed")
        if response.artifacts is None:
            raise RuntimeError("renderer returned no artifacts")

        render_blobs = response.artifacts.render_blobs_base64 or {}
        object_store_keys = response.artifacts.object_store_keys or {}
        output_key = next(
            (key for key in render_blobs if Path(key).name == output_name),
            None,
        )
        object_store_path = next(
            (
                path
                for path, value in object_store_keys.items()
                if Path(path).name == output_name or Path(value).name == output_name
            ),
            None,
        )
        object_store_key = (
            object_store_keys[object_store_path] if object_store_path else None
        )

        if object_store_key:
            client = _stress_heatmap_s3_client()
            if client is None:
                raise RuntimeError(
                    "renderer returned an object-store-backed heatmap but S3 is not configured"
                )
            with tempfile.NamedTemporaryFile(suffix=Path(output_name).suffix) as tmp:
                client.download_file(object_store_key, tmp.name)
                image_bytes = Path(tmp.name).read_bytes()
        elif output_key is not None:
            image_bytes = base64.b64decode(render_blobs[output_key])
        else:
            raise RuntimeError("renderer returned no stress heatmap blob")

        return RenderedStressHeatmap(
            image_bytes=image_bytes,
            object_store_key=object_store_key,
        )
