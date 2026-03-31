from __future__ import annotations

import shutil
import sys
import tempfile
from pathlib import Path

import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from build123d import Align, Box, Compound, Cylinder, Location, Sphere

from shared.git_utils import repo_revision
from shared.models.schemas import BenchmarkDefinition, CompoundMetadata, PartMetadata
from shared.workers.loader import load_component_from_script
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)

_RENDER_ROLE_WITH_SCRIPT = {
    "benchmark_reviewer",
    "engineer_execution_reviewer",
}
_RENDER_ROLE_WITH_DEFINITION_PREVIEW = {
    "engineer_planner",
    "engineer_plan_reviewer",
    "engineer_coder",
}
_NO_RENDER_ROLE = {
    "benchmark_plan_reviewer",
    "benchmark_coder",
}

_CONTEXT_VIEW_ANGLES = (
    ("context_view_01", -35.0, 45.0),
    ("context_view_02", -35.0, 135.0),
)


def _patch_ambient_vtk_renderer() -> None:
    """Force build123d preview imports to use the ambient display path."""
    from vtkmodules.vtkRenderingCore import vtkRenderWindow

    import shared.rendering as shared_rendering

    def _create_render_window() -> vtkRenderWindow:
        window = vtkRenderWindow()
        window.SetOffScreenRendering(True)
        return window

    shared_rendering.configure_headless_vtk_egl = lambda: None
    shared_rendering.create_headless_vtk_render_window = _create_render_window


def _load_benchmark_definition(artifact_dir: Path) -> BenchmarkDefinition:
    definition_path = artifact_dir / "benchmark_definition.yaml"
    if not definition_path.exists():
        raise FileNotFoundError(f"benchmark_definition.yaml missing in {artifact_dir}")
    data = yaml.safe_load(definition_path.read_text(encoding="utf-8")) or {}
    return BenchmarkDefinition.model_validate(data)


def _build_moved_object_component(definition: BenchmarkDefinition) -> Compound:
    moved = definition.moved_object
    radius_range = moved.static_randomization.radius
    radius = float(max(radius_range)) if radius_range else 10.0
    shape = moved.shape.strip().lower()

    if shape == "sphere":
        part = Sphere(radius, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    elif shape in {"box", "cube"}:
        edge = radius * 2.0
        part = Box(edge, edge, edge, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    elif shape == "cylinder":
        part = Cylinder(
            radius=radius,
            height=radius * 2.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        )
    else:
        raise ValueError(
            f"Unsupported moved_object.shape '{moved.shape}'. "
            "Expected sphere, box, cube, or cylinder."
        )

    part = part.move(Location(tuple(moved.start_position)))
    part.label = moved.label
    part.metadata = PartMetadata(material_id=moved.material_id, fixed=False)

    preview = Compound(label=f"{moved.label}_preview", children=[part])
    preview.metadata = CompoundMetadata(fixed=False)
    return preview


def _load_preview_component(
    artifact_dir: Path, definition: BenchmarkDefinition, role_name: str
) -> Compound:
    script_path = artifact_dir / "script.py"
    if role_name in _RENDER_ROLE_WITH_SCRIPT and script_path.exists():
        return load_component_from_script(script_path, session_root=artifact_dir)
    return _build_moved_object_component(definition)


def _role_name_for_artifact(artifact_dir: Path) -> str:
    return artifact_dir.parent.name


def _render_context_views(scene, output_dir: Path) -> list[Path]:
    from worker_heavy.utils.build123d_rendering import render_preview_scene

    output_dir.mkdir(parents=True, exist_ok=True)
    rendered_paths: list[Path] = []

    with tempfile.TemporaryDirectory() as tmpdir:
        temp_root = Path(tmpdir)
        for stem, pitch, yaw in _CONTEXT_VIEW_ANGLES:
            jpg_path = render_preview_scene(
                scene,
                pitch=pitch,
                yaw=yaw,
                output_dir=temp_root,
                width=640,
                height=480,
            )
            png_path = output_dir / f"{stem}.png"
            with Image.open(jpg_path) as image:
                image.save(png_path, format="PNG")
            rendered_paths.append(png_path)

    return rendered_paths


def _manifest_for_render_paths(
    *,
    artifact_dir: Path,
    render_paths: list[Path],
    existing_manifest: RenderManifest | None,
) -> RenderManifest:
    revision = repo_revision(ROOT)
    if not revision:
        raise RuntimeError("Unable to determine repository revision for seed renders")

    environment_version = None
    if existing_manifest is not None and existing_manifest.environment_version:
        environment_version = existing_manifest.environment_version
    elif (artifact_dir / "benchmark_assembly_definition.yaml").exists():
        try:
            assembly = (
                yaml.safe_load(
                    (artifact_dir / "benchmark_assembly_definition.yaml").read_text(
                        encoding="utf-8"
                    )
                )
                or {}
            )
            environment_version = str(assembly.get("version") or "").strip() or None
        except Exception:
            environment_version = None
    if environment_version is None and existing_manifest is not None:
        environment_version = existing_manifest.environment_version

    render_rel_paths = [
        str(path.relative_to(artifact_dir)).replace("\\", "/") for path in render_paths
    ]
    artifacts = {
        rel_path: RenderArtifactMetadata(
            modality="rgb",
            group_key="context",
            siblings=RenderSiblingPaths(rgb=rel_path),
        )
        for rel_path in render_rel_paths
    }

    return RenderManifest(
        episode_id=artifact_dir.name,
        worker_session_id=artifact_dir.name,
        revision=revision,
        environment_version=environment_version,
        preview_evidence_paths=render_rel_paths,
        artifacts=artifacts,
    )


def update_seed_artifact_renders(artifact_dir: Path) -> list[str]:
    artifact_dir = Path(artifact_dir)
    role_name = _role_name_for_artifact(artifact_dir)
    renders_dir = artifact_dir / "renders"
    images_dir = renders_dir / "images"

    if role_name in _NO_RENDER_ROLE:
        if renders_dir.exists():
            shutil.rmtree(renders_dir)
        return []

    if role_name not in _RENDER_ROLE_WITH_SCRIPT | _RENDER_ROLE_WITH_DEFINITION_PREVIEW:
        return []

    definition = _load_benchmark_definition(artifact_dir)
    component = _load_preview_component(artifact_dir, definition, role_name)

    existing_manifest = None
    manifest_path = images_dir / "render_manifest.json"
    if manifest_path.exists():
        try:
            existing_manifest = RenderManifest.model_validate_json(
                manifest_path.read_text(encoding="utf-8")
            )
        except Exception:
            existing_manifest = None

    if images_dir.exists():
        shutil.rmtree(images_dir)
    images_dir.mkdir(parents=True, exist_ok=True)

    _patch_ambient_vtk_renderer()
    from worker_heavy.utils.build123d_rendering import collect_preview_scene

    with tempfile.TemporaryDirectory() as mesh_tmpdir:
        mesh_root = Path(mesh_tmpdir) / "meshes"
        mesh_root.mkdir(parents=True, exist_ok=True)

        scene = collect_preview_scene(
            component,
            objectives=definition,
            workspace_root=artifact_dir,
            smoke_test_mode=False,
            mesh_root=mesh_root,
        )

        rendered_paths = _render_context_views(scene, images_dir)
        manifest = _manifest_for_render_paths(
            artifact_dir=artifact_dir,
            render_paths=rendered_paths,
            existing_manifest=existing_manifest,
        )
        manifest_path.write_text(manifest.model_dump_json(indent=2), encoding="utf-8")

    return [
        str(path.relative_to(artifact_dir)).replace("\\", "/")
        for path in rendered_paths
    ]
