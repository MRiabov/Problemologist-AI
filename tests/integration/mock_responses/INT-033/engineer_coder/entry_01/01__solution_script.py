from __future__ import annotations

import base64
import hashlib
import os
from pathlib import Path
from time import time

from build123d import Box

from shared.enums import ManufacturingMethod
from shared.models.schemas import PartMetadata
from shared.models.simulation import MultiRunResult, SimulationMetrics, SimulationResult
from shared.workers.schema import RenderArtifactMetadata, RenderManifest, ReviewManifest, ValidationResultRecord


_PNG_BYTES = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO+yX2sAAAAASUVORK5CYII="
)


def build():
    part = Box(1, 1, 1).translate((8, 0, 0.5))
    part.label = "projectile_ball"
    part.metadata = PartMetadata(
        material_id="abs",
        fixed=False,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return part


def _write_png(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_PNG_BYTES)


def _seed_execution_review_handoff() -> None:
    script_path = Path(__file__)
    script_sha256 = hashlib.sha256(script_path.read_bytes()).hexdigest()
    revision = os.environ["REPO_REVISION"]
    session_id = os.environ["SESSION_ID"]
    episode_id = os.environ.get("EPISODE_ID")
    seed_ts = time()

    benchmark_review_path = Path(".manifests/benchmark_review_manifest.json")
    benchmark_review = ReviewManifest.model_validate_json(
        benchmark_review_path.read_text(encoding="utf-8")
    )

    preview_evidence_paths = [
        "renders/render_e45_a45.png",
        "renders/render_e45_a45_depth.png",
        "renders/render_e45_a45_segmentation.png",
    ]
    for render_path in preview_evidence_paths:
        _write_png(Path(render_path))

    render_manifest = RenderManifest(
        episode_id=episode_id,
        worker_session_id=session_id,
        revision=revision,
        preview_evidence_paths=preview_evidence_paths,
        artifacts={
            "renders/render_e45_a45.png": RenderArtifactMetadata(modality="rgb"),
            "renders/render_e45_a45_depth.png": RenderArtifactMetadata(
                modality="depth"
            ),
            "renders/render_e45_a45_segmentation.png": RenderArtifactMetadata(
                modality="segmentation"
            ),
        },
    )

    validation = ValidationResultRecord(
        success=True,
        message="Validation completed",
        timestamp=seed_ts,
        script_path="solution_script.py",
        script_sha256=script_sha256,
        verification_result=MultiRunResult(
            num_scenes=1,
            success_count=1,
            success_rate=1.0,
            is_consistent=True,
            individual_results=[SimulationMetrics(success=True)],
            fail_reasons=[],
            scene_build_count=1,
            backend_run_count=1,
            batched_execution=True,
        ),
    )
    simulation = SimulationResult(
        success=True,
        summary="Goal achieved in green zone.",
        render_paths=preview_evidence_paths,
        confidence="high",
    )
    manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage="engineering_execution_reviewer",
        session_id=session_id,
        revision=revision,
        episode_id=episode_id,
        worker_session_id=session_id,
        benchmark_episode_id=benchmark_review.benchmark_episode_id,
        benchmark_worker_session_id=benchmark_review.benchmark_worker_session_id,
        benchmark_revision=revision,
        solution_revision=revision,
        script_path="solution_script.py",
        script_sha256=script_sha256,
        validation_success=True,
        validation_timestamp=seed_ts,
        simulation_success=True,
        simulation_summary="Goal achieved in green zone.",
        simulation_timestamp=seed_ts,
        goal_reached=True,
        renders=preview_evidence_paths,
        preview_evidence_paths=preview_evidence_paths,
        mjcf_path="renders/scene.xml",
        objectives_path="benchmark_definition.yaml",
        assembly_definition_path="assembly_definition.yaml",
    )

    Path("validation_results.json").write_text(
        validation.model_dump_json(indent=2),
        encoding="utf-8",
    )
    Path("simulation_result.json").write_text(
        simulation.model_dump_json(indent=2),
        encoding="utf-8",
    )
    manifests_dir = Path(".manifests")
    manifests_dir.mkdir(parents=True, exist_ok=True)
    (manifests_dir / "engineering_execution_review_manifest.json").write_text(
        manifest.model_dump_json(indent=2),
        encoding="utf-8",
    )
    Path("renders/render_manifest.json").write_text(
        render_manifest.model_dump_json(indent=2),
        encoding="utf-8",
    )
    print("handoff seeded")


result = build()

if __name__ == "__main__":
    _seed_execution_review_handoff()
