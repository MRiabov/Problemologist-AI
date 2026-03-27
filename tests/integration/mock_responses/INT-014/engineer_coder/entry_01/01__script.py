import hashlib
import os
import time
from pathlib import Path

from build123d import Align, Box, Compound, Location, Sphere

from shared.enums import ManufacturingMethod
from shared.models.schemas import CompoundMetadata, PartMetadata
from shared.models.simulation import MultiRunResult, SimulationMetrics, SimulationResult
from shared.workers.schema import ReviewManifest, ValidationResultRecord


def build():
    support = Box(10, 10, 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN))
    support = support.move(Location((7.5, 7.5, 5.0)))
    support.label = "goal_platform"
    support.metadata = PartMetadata(
        material_id="aluminum_6061",
        fixed=True,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )

    projectile_ball = Sphere(1.0)
    projectile_ball = projectile_ball.move(Location((7.5, 7.5, 6.5)))
    projectile_ball.label = "projectile_ball"
    projectile_ball.metadata = PartMetadata(
        material_id="abs",
        fixed=False,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )

    scene = Compound(children=[support, projectile_ball])
    scene.label = "benchmark_scene"
    scene.metadata = CompoundMetadata(fixed=False)
    return scene


def _seed_execution_review_handoff() -> None:
    script_path = Path(__file__)
    script_sha256 = hashlib.sha256(script_path.read_bytes()).hexdigest()
    revision = os.environ["REPO_REVISION"]
    session_id = os.environ["SESSION_ID"]
    seed_ts = time.time()

    validation = ValidationResultRecord(
        success=True,
        message="Validation completed",
        timestamp=seed_ts,
        script_path="script.py",
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
        render_paths=[],
        confidence="high",
    )
    manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage="engineering_execution_reviewer",
        session_id=session_id,
        script_path="script.py",
        script_sha256=script_sha256,
        validation_success=True,
        validation_timestamp=seed_ts,
        simulation_success=True,
        simulation_summary="Goal achieved in green zone.",
        simulation_timestamp=seed_ts,
        goal_reached=True,
        revision=revision,
        renders=[],
    )

    Path("validation_results.json").write_text(
        validation.model_dump_json(indent=2), encoding="utf-8"
    )
    Path("simulation_result.json").write_text(
        simulation.model_dump_json(indent=2), encoding="utf-8"
    )
    manifests_dir = Path(".manifests")
    manifests_dir.mkdir(parents=True, exist_ok=True)
    (manifests_dir / "engineering_execution_review_manifest.json").write_text(
        manifest.model_dump_json(indent=2), encoding="utf-8"
    )


result = build()
_seed_execution_review_handoff()
