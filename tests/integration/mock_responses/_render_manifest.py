from __future__ import annotations

import os
from pathlib import Path

from shared.git_utils import repo_revision
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)


def write_render_manifest() -> None:
    renders_dir = Path("renders")
    render_paths = sorted(
        str(path).replace("\\", "/")
        for path in renders_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in {".png", ".jpg", ".jpeg", ".mp4"}
    )
    if not render_paths:
        return

    revision = os.environ.get("REPO_REVISION") or repo_revision(Path.cwd())
    if not revision:
        revision = repo_revision(Path(__file__).resolve().parents[3])
    if not revision:
        raise RuntimeError("Unable to determine repository revision for render manifest")

    manifest = RenderManifest(
        episode_id=os.environ.get("EPISODE_ID"),
        worker_session_id=os.environ.get("SESSION_ID"),
        revision=revision,
        preview_evidence_paths=render_paths,
        artifacts={
            path: RenderArtifactMetadata(
                modality="unknown" if path.endswith(".mp4") else "rgb",
                siblings=(
                    RenderSiblingPaths(
                        rgb=path,
                        svg=str(Path(path).with_suffix(".svg")),
                        dxf=str(Path(path).with_suffix(".dxf")),
                    )
                    if path.endswith((".png", ".jpg", ".jpeg"))
                    else RenderSiblingPaths()
                ),
            )
            for path in render_paths
        },
    )

    renders_dir.mkdir(parents=True, exist_ok=True)
    (renders_dir / "render_manifest.json").write_text(
        manifest.model_dump_json(indent=2),
        encoding="utf-8",
    )
