from __future__ import annotations

import asyncio
from pathlib import Path

import structlog
import yaml

from controller.observability.middleware_helper import broadcast_file_update
from shared.models.schemas import AssemblyDefinition
from shared.rendering import build_render_manifest
from shared.workers.schema import RenderArtifactMetadata, RenderSiblingPaths

logger = structlog.get_logger(__name__)


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
    Fail closed if the benchmark review render bundle is missing.

    The benchmark review path relies on a canonical render bundle during
    startup. The bundle must already include the preview images, sidecars, and
    render manifest. This helper does not synthesize any fallback assets.
    """

    canonical_render_paths = sorted(
        dict.fromkeys(
            path
            for path in (_normalize_render_path(path) for path in render_paths)
            if path and Path(path).suffix.lower() in {".png", ".jpg", ".jpeg"}
        )
    )
    if not canonical_render_paths:
        raise FileNotFoundError("benchmark review render bundle has no render paths")

    existing_paths: set[str] = set()
    try:
        render_entries = await asyncio.wait_for(
            worker_client.list_files("/renders/"),
            timeout=5.0,
        )
        existing_paths = {
            str(entry.path).lstrip("/") for entry in render_entries if not entry.is_dir
        }
    except Exception as exc:
        logger.warning(
            "benchmark_review_render_seed_list_failed",
            session_id=session_id,
            error=str(exc),
        )

    missing_paths: list[str] = []
    for render_path in canonical_render_paths:
        svg_path = str(Path(render_path).with_suffix(".svg"))
        dxf_path = str(Path(render_path).with_suffix(".dxf"))
        for candidate in (render_path, svg_path, dxf_path):
            if candidate not in existing_paths:
                missing_paths.append(candidate)

    if missing_paths:
        raise FileNotFoundError(
            "benchmark review render bundle is missing required paths: "
            f"{sorted(set(missing_paths))}"
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
