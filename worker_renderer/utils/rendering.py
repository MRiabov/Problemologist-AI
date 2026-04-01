from __future__ import annotations

import os
import uuid
from pathlib import Path

from shared.git_utils import repo_revision
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)


def _derived_episode_id(session_id: str | None) -> str | None:
    if not session_id:
        return None
    try:
        return str(uuid.UUID(session_id))
    except Exception:
        return str(uuid.uuid5(uuid.NAMESPACE_DNS, session_id))


def _workspace_environment_version(workspace_root: Path | None) -> str | None:
    if workspace_root is None:
        return None

    for rel_path in ("benchmark_assembly_definition.yaml", "assembly_definition.yaml"):
        definition_path = workspace_root / rel_path
        if not definition_path.exists():
            continue
        try:
            import yaml

            from shared.models.schemas import AssemblyDefinition

            definition = AssemblyDefinition.model_validate(
                yaml.safe_load(definition_path.read_text(encoding="utf-8")) or {}
            )
            version = str(definition.version).strip()
            return version or None
        except Exception:
            continue
    return None


def build_render_manifest(
    artifacts: dict[str, RenderArtifactMetadata],
    *,
    workspace_root: Path | None = None,
    episode_id: str | None = None,
    worker_session_id: str | None = None,
    revision: str | None = None,
    environment_version: str | None = None,
    preview_evidence_paths: list[str] | None = None,
) -> RenderManifest:
    resolved_revision = revision
    if not resolved_revision:
        resolved_revision = os.getenv("REPO_REVISION")
    if not resolved_revision:
        resolved_revision = repo_revision(Path.cwd())
    if not resolved_revision and workspace_root is not None:
        resolved_revision = repo_revision(workspace_root)
    if not resolved_revision:
        resolved_revision = repo_revision(Path(__file__).resolve().parents[2])

    if environment_version is None:
        environment_version = _workspace_environment_version(workspace_root)

    if preview_evidence_paths is None:
        preview_evidence_paths = sorted(
            path
            for path in artifacts
            if Path(path).suffix.lower() in {".png", ".jpg", ".jpeg", ".mp4"}
        )

    return RenderManifest(
        episode_id=_derived_episode_id(episode_id),
        worker_session_id=worker_session_id,
        revision=resolved_revision,
        environment_version=environment_version,
        preview_evidence_paths=preview_evidence_paths,
        artifacts=artifacts,
    )


def _render_group_key(render_path: str) -> tuple[str, str]:
    filename = Path(render_path).name
    stem = Path(render_path).stem
    if filename.endswith("_depth.png"):
        return stem.removesuffix("_depth"), "depth"
    if filename.endswith("_segmentation.png"):
        return stem.removesuffix("_segmentation"), "segmentation"
    if filename.endswith(".mp4"):
        return stem, "unknown"
    return stem, "rgb"


def _infer_render_artifact_metadata(
    render_path: str,
    *,
    existing: RenderArtifactMetadata | None = None,
) -> RenderArtifactMetadata:
    group_key, modality = _render_group_key(render_path)
    rel_path = Path(render_path)
    render_dir = rel_path.parent
    metadata = RenderArtifactMetadata(
        modality=modality,
        group_key=group_key,
        siblings=RenderSiblingPaths(
            rgb=str(render_dir / f"{group_key}.png"),
            depth=str(render_dir / f"{group_key}_depth.png"),
            segmentation=str(render_dir / f"{group_key}_segmentation.png"),
        ),
    )

    if modality == "depth":
        metadata.depth_interpretation = (
            existing.depth_interpretation
            if existing and existing.depth_interpretation
            else "Camera-space depth in meters. False-color pixels are scaled "
            "from the build123d/VTK preview renderer's linear depth buffer; "
            "see depth_min_m and depth_max_m for the metric range."
        )
    elif modality == "segmentation" and existing and existing.segmentation_legend:
        metadata.segmentation_legend = list(existing.segmentation_legend)
    elif modality == "segmentation" and existing and not existing.segmentation_legend:
        metadata.segmentation_legend = []

    if existing is not None:
        if existing.depth_min_m is not None:
            metadata.depth_min_m = existing.depth_min_m
        if existing.depth_max_m is not None:
            metadata.depth_max_m = existing.depth_max_m
        if existing.depth_interpretation and modality != "depth":
            metadata.depth_interpretation = existing.depth_interpretation
        if existing.segmentation_legend and modality != "segmentation":
            metadata.segmentation_legend = list(existing.segmentation_legend)

    return metadata


def normalize_render_manifest(
    *,
    render_paths: list[str],
    workspace_root: Path | None = None,
    existing_manifest: RenderManifest | None = None,
    episode_id: str | None = None,
    worker_session_id: str | None = None,
    revision: str | None = None,
    environment_version: str | None = None,
    preview_evidence_paths: list[str] | None = None,
) -> RenderManifest:
    existing_artifacts = dict(existing_manifest.artifacts) if existing_manifest else {}
    normalized_artifacts: dict[str, RenderArtifactMetadata] = {}
    for render_path in render_paths:
        suffix = Path(render_path).suffix.lower()
        if suffix not in {".png", ".jpg", ".jpeg", ".mp4"}:
            continue
        normalized_artifacts[render_path] = _infer_render_artifact_metadata(
            render_path, existing=existing_artifacts.get(render_path)
        )

    resolved_revision = revision
    if not resolved_revision and existing_manifest is not None:
        resolved_revision = existing_manifest.revision
    resolved_environment_version = environment_version
    if resolved_environment_version is None and existing_manifest is not None:
        resolved_environment_version = existing_manifest.environment_version

    return build_render_manifest(
        normalized_artifacts,
        workspace_root=workspace_root,
        episode_id=episode_id,
        worker_session_id=worker_session_id,
        revision=resolved_revision,
        environment_version=resolved_environment_version,
        preview_evidence_paths=preview_evidence_paths,
    )


def _is_benchmark_role(agent_role: str | None) -> bool:
    return bool(agent_role and agent_role.startswith("benchmark_"))


def select_single_preview_render_subdir(
    workspace_root: Path, *, agent_role: str | None = None
) -> str:
    if agent_role:
        return (
            "benchmark_renders"
            if _is_benchmark_role(agent_role)
            else "engineer_renders"
        )
    return (
        "engineer_renders"
        if (workspace_root / "assembly_definition.yaml").exists()
        else "benchmark_renders"
    )


def select_static_preview_render_subdir(
    workspace_root: Path, *, agent_role: str | None = None
) -> str:
    if agent_role:
        return (
            "benchmark_renders"
            if _is_benchmark_role(agent_role)
            else "final_preview_renders"
        )
    return (
        "final_preview_renders"
        if (workspace_root / "assembly_definition.yaml").exists()
        else "benchmark_renders"
    )
