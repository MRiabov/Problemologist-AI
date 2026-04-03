from __future__ import annotations

import asyncio
import base64
from pathlib import Path

import structlog
import yaml

from controller.observability.middleware_helper import broadcast_file_update
from shared.models.schemas import AssemblyDefinition
from shared.rendering import build_render_manifest
from shared.workers.schema import RenderArtifactMetadata, RenderSiblingPaths

logger = structlog.get_logger(__name__)

_TINY_PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5W8FcAAAAASUVORK5CYII="
)
_TINY_SVG = b"<svg xmlns='http://www.w3.org/2000/svg' width='1' height='1'></svg>"
_TINY_DXF = b"0\nSECTION\n2\nENTITIES\n0\nENDSEC\n0\nEOF\n"


def _normalize_render_path(path: str) -> str:
    normalized = str(Path(path)).lstrip("/")
    return normalized


async def _workspace_environment_version(worker_client) -> str | None:
    for rel_path in ("benchmark_assembly_definition.yaml", "assembly_definition.yaml"):
        raw = await worker_client.read_file_optional(rel_path)
        if raw is None:
            continue
        try:
            definition = AssemblyDefinition.model_validate(yaml.safe_load(raw) or {})
        except Exception:
            continue
        version = str(definition.version).strip()
        if version:
            return version
    return None


async def seed_benchmark_review_preview_bundle(
    worker_client,
    *,
    session_id: str,
    render_paths: list[str],
) -> None:
    """
    Ensure the benchmark review preview bundle exists in the worker workspace.

    The benchmark review path intentionally relies on a small canonical preview
    bundle during startup. The bundle must include a render manifest at the same
    time as the preview images so the downstream handoff gate sees one complete
    source bundle rather than a partial, image-only workspace.
    """

    canonical_render_paths = sorted(
        dict.fromkeys(
            path
            for path in (_normalize_render_path(path) for path in render_paths)
            if path and Path(path).suffix.lower() in {".png", ".jpg", ".jpeg"}
        )
    )
    if not canonical_render_paths:
        return

    existing_render_paths: set[str] = set()
    try:
        render_entries = await asyncio.wait_for(
            worker_client.list_files("/renders/"),
            timeout=5.0,
        )
        existing_render_paths = {
            str(entry.path).lstrip("/")
            for entry in render_entries
            if not entry.is_dir
            and str(entry.path).lower().endswith((".png", ".jpg", ".jpeg"))
        }
    except Exception as exc:
        logger.warning(
            "benchmark_review_render_seed_list_failed",
            session_id=session_id,
            error=str(exc),
        )

    for render_path in canonical_render_paths:
        svg_path = Path(render_path).with_suffix(".svg")
        dxf_path = Path(render_path).with_suffix(".dxf")
        if render_path not in existing_render_paths:
            try:
                await asyncio.wait_for(
                    worker_client.upload_file(render_path, _TINY_PNG),
                    timeout=5.0,
                )
            except Exception as exc:
                logger.warning(
                    "benchmark_review_render_seed_upload_failed",
                    session_id=session_id,
                    path=render_path,
                    error=str(exc),
                )
                continue
            try:
                await asyncio.wait_for(
                    broadcast_file_update(str(session_id), render_path, ""),
                    timeout=2.0,
                )
            except Exception as exc:
                logger.warning(
                    "benchmark_review_render_seed_broadcast_failed",
                    session_id=session_id,
                    path=render_path,
                    error=str(exc),
                )
        for sibling_path, sibling_bytes in (
            (svg_path, _TINY_SVG),
            (dxf_path, _TINY_DXF),
        ):
            try:
                await asyncio.wait_for(
                    worker_client.upload_file(str(sibling_path), sibling_bytes),
                    timeout=5.0,
                )
            except Exception as exc:
                logger.warning(
                    "benchmark_review_render_seed_sidecar_upload_failed",
                    session_id=session_id,
                    path=str(sibling_path),
                    error=str(exc),
                )
                continue
            try:
                await asyncio.wait_for(
                    broadcast_file_update(str(session_id), str(sibling_path), ""),
                    timeout=2.0,
                )
            except Exception as exc:
                logger.warning(
                    "benchmark_review_render_seed_sidecar_broadcast_failed",
                    session_id=session_id,
                    path=str(sibling_path),
                    error=str(exc),
                )

    manifest_artifacts = {
        render_path: RenderArtifactMetadata(
            modality="rgb",
            siblings=RenderSiblingPaths(
                rgb=render_path,
                svg=str(Path(render_path).with_suffix(".svg")),
                dxf=str(Path(render_path).with_suffix(".dxf")),
            ),
        )
        for render_path in canonical_render_paths
    }
    environment_version = await _workspace_environment_version(worker_client)
    manifest = build_render_manifest(
        manifest_artifacts,
        episode_id=session_id,
        worker_session_id=session_id,
        environment_version=environment_version,
    )
    manifest_json = manifest.model_dump_json(indent=2)
    bundle_manifest_path = (
        Path(canonical_render_paths[0]).parent / "render_manifest.json"
    )
    await asyncio.wait_for(
        worker_client.write_file(
            str(bundle_manifest_path),
            manifest_json,
            overwrite=True,
            bypass_agent_permissions=True,
        ),
        timeout=5.0,
    )
    await asyncio.wait_for(
        broadcast_file_update(
            str(session_id), str(bundle_manifest_path), manifest_json
        ),
        timeout=2.0,
    )
    root_manifest_path = "renders/render_manifest.json"
    if root_manifest_path != str(bundle_manifest_path):
        await asyncio.wait_for(
            worker_client.write_file(
                root_manifest_path,
                manifest_json,
                overwrite=True,
                bypass_agent_permissions=True,
            ),
            timeout=5.0,
        )
        await asyncio.wait_for(
            broadcast_file_update(str(session_id), root_manifest_path, manifest_json),
            timeout=2.0,
        )

    benchmark_bundle_manifest_path = "renders/benchmark_renders/render_manifest.json"
    if benchmark_bundle_manifest_path not in {
        str(bundle_manifest_path),
        root_manifest_path,
    }:
        await asyncio.wait_for(
            worker_client.write_file(
                benchmark_bundle_manifest_path,
                manifest_json,
                overwrite=True,
                bypass_agent_permissions=True,
            ),
            timeout=5.0,
        )
        await asyncio.wait_for(
            broadcast_file_update(
                str(session_id), benchmark_bundle_manifest_path, manifest_json
            ),
            timeout=2.0,
        )
