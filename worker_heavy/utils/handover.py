import os
import subprocess
from hashlib import sha256
from pathlib import Path

import structlog
import yaml
from build123d import Compound, export_step

from shared.models.schemas import ObjectivesYaml
from shared.models.simulation import SimulationResult
from shared.workers.schema import ReviewManifest, ValidationResultRecord
from worker_heavy.utils.dfm import validate_and_price_assembly
from worker_heavy.workbenches.config import load_config

logger = structlog.get_logger(__name__)


def _sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _goal_reached(summary: str) -> bool:
    s = (summary or "").lower()
    return "goal achieved" in s or "green zone" in s or "goal zone" in s


def _latest_git_revision(cwd: Path) -> str | None:
    try:
        return (
            subprocess.check_output(
                ["git", "-C", str(cwd), "rev-parse", "HEAD"],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            .strip()
            .lower()
        )
    except Exception:
        return None


def submit_for_review(
    component: Compound,
    cwd: Path = Path(),
    session_id: str | None = None,
    reviewer_stage: str = "engineering_execution_reviewer",
):
    """
    Standardized handover from Coder to Reviewer.
    Logic:
    - Persist temporary assets to the /renders/ folder.
    - Trigger a LangGraph event or update shared state for the Reviewer node.
    """
    logger.info(
        "handover_started", cwd=str(cwd), files=os.listdir(cwd), session_id=session_id
    )

    renders_dir = cwd / os.getenv("RENDERS_DIR", "renders")
    manifests_dir = cwd / ".manifests"

    # 1. Validate mandatory base files (INT-005)

    # plan.md
    plan_path = cwd / "plan.md"
    if plan_path.exists():
        from .file_validation import validate_plan_md_structure

        plan_content = plan_path.read_text(encoding="utf-8")
        lowered = plan_content.lower()
        plan_type = "benchmark" if "learning objective" in lowered else "engineering"
        is_valid, errors = validate_plan_md_structure(
            plan_content, plan_type=plan_type, session_id=session_id
        )
        if not is_valid:
            logger.error(
                "plan_md_invalid",
                plan_type=plan_type,
                violations=errors,
                session_id=session_id,
            )
            raise ValueError(f"plan.md invalid: {errors}")
    else:
        logger.error("plan_md_missing", session_id=session_id)
        raise ValueError("plan.md is missing (required for submission)")

    # todo.md
    todo_path = cwd / "todo.md"
    if todo_path.exists():
        from shared.workers.markdown_validator import validate_todo_md

        todo_content = todo_path.read_text(encoding="utf-8")
        todo_result = validate_todo_md(todo_content, require_completion=True)
        if not todo_result.is_valid:
            logger.error(
                "todo_md_invalid",
                violations=todo_result.violations,
                session_id=session_id,
            )
            raise ValueError(f"todo.md invalid: {todo_result.violations}")
    else:
        logger.error("todo_md_missing", session_id=session_id)
        raise ValueError("todo.md is missing (required for submission)")

    # objectives.yaml
    objectives_path = cwd / "objectives.yaml"
    if objectives_path.exists():
        from .file_validation import validate_objectives_yaml

        objectives_content = objectives_path.read_text(encoding="utf-8")
        is_valid, result = validate_objectives_yaml(
            objectives_content, session_id=session_id
        )
        if not is_valid:
            logger.error(
                "objectives_yaml_invalid", errors=result, session_id=session_id
            )
            raise ValueError(f"objectives.yaml invalid: {result}")

        # INT-015: Verify immutability
        from .file_validation import validate_immutability

        is_immutable, error_msg = validate_immutability(
            objectives_path, session_id=session_id
        )
        if not is_immutable:
            logger.error("objectives_yaml_modified", session_id=session_id)
            raise ValueError(f"objectives.yaml violation: {error_msg}")
    else:
        logger.error("objectives_yaml_missing", session_id=session_id)
        raise ValueError("objectives.yaml is missing (required for submission)")

    # assembly_definition.yaml
    cost_path = cwd / "assembly_definition.yaml"
    if cost_path.exists():
        from .file_validation import validate_assembly_definition_yaml

        cost_content = cost_path.read_text(encoding="utf-8")
        is_valid, estimation = validate_assembly_definition_yaml(
            cost_content, session_id=session_id
        )
        if not is_valid:
            logger.error(
                "assembly_definition_yaml_invalid",
                errors=estimation,
                session_id=session_id,
            )
            raise ValueError(f"assembly_definition.yaml invalid: {estimation}")
    else:
        logger.error("assembly_definition_yaml_missing", session_id=session_id)
        raise ValueError(
            "assembly_definition.yaml is missing (required for submission)"
        )

    # 2. Verify prior validation (INT-018) for current script revision
    script_path = cwd / "script.py"
    if not script_path.exists():
        logger.error("script_missing_for_handover", session_id=session_id)
        raise ValueError("script.py is missing (required for submission)")
    script_mtime = script_path.stat().st_mtime
    script_sha256 = _sha256_file(script_path)

    validation_results_path = cwd / "validation_results.json"
    if not validation_results_path.exists():
        logger.error("prior_validation_missing", session_id=session_id)
        raise ValueError(
            "Prior validation missing. Call /benchmark/validate before submission."
        )
    validation_record = ValidationResultRecord.model_validate_json(
        validation_results_path.read_text(encoding="utf-8")
    )
    if not validation_record.success:
        logger.error("prior_validation_failed", session_id=session_id)
        raise ValueError(
            "Prior validation failed. Fix validation errors before submission."
        )
    if validation_results_path.stat().st_mtime < script_mtime:
        logger.error("prior_validation_stale_for_script", session_id=session_id)
        raise ValueError(
            "Prior validation is stale for current script revision. Re-run validate."
        )

    # 2b. Verify prior simulation for current script revision and objective success.
    simulation_results_path = cwd / "simulation_result.json"
    if not simulation_results_path.exists():
        logger.error("prior_simulation_missing", session_id=session_id)
        raise ValueError("Prior simulation missing. Call /benchmark/simulate first.")
    simulation_result = SimulationResult.model_validate_json(
        simulation_results_path.read_text(encoding="utf-8")
    )
    if not simulation_result.success:
        logger.error(
            "prior_simulation_failed",
            summary=simulation_result.summary,
            session_id=session_id,
        )
        raise ValueError(
            "Prior simulation failed. Submission requires a successful simulation."
        )
    if not _goal_reached(simulation_result.summary):
        logger.error(
            "goal_not_reached_in_simulation",
            summary=simulation_result.summary,
            session_id=session_id,
        )
        raise ValueError(
            "Simulation did not report goal completion (green/goal zone reached)."
        )
    if simulation_results_path.stat().st_mtime < script_mtime:
        logger.error("prior_simulation_stale_for_script", session_id=session_id)
        raise ValueError(
            "Prior simulation is stale for current script revision. Re-run simulate."
        )

    # 3. Perform DFM + Geometry Checks (INT-019)
    renders_dir.mkdir(parents=True, exist_ok=True)
    manifests_dir.mkdir(parents=True, exist_ok=True)
    dfm_config = load_config()

    objectives_data = yaml.safe_load(objectives_path.read_text())
    objectives_model = ObjectivesYaml(**objectives_data)
    build_zone = objectives_model.objectives.build_zone
    constraints = objectives_model.constraints

    manufactured_labels = {part.part_name for part in estimation.manufactured_parts}
    if reviewer_stage != "benchmark_reviewer" and manufactured_labels:
        method = estimation.manufactured_parts[0].manufacturing_method
        validation_result = validate_and_price_assembly(
            component,
            dfm_config,
            part_labels=manufactured_labels,
            build_zone=build_zone,
            session_id=session_id,
            default_method=method,
        )

        if not validation_result.is_manufacturable:
            logger.warning(
                "submission_dfm_failed",
                violations=validation_result.violations,
                session_id=session_id,
            )
            raise ValueError(
                f"Submission rejected (DFM): {validation_result.violations}"
            )

        if constraints:
            if (
                constraints.max_unit_cost
                and validation_result.unit_cost > constraints.max_unit_cost
            ):
                msg = f"Unit cost ${validation_result.unit_cost:.2f} exceeds limit ${constraints.max_unit_cost:.2f}"
                logger.error(
                    "submission_cost_limit_exceeded",
                    cost=validation_result.unit_cost,
                    limit=constraints.max_unit_cost,
                    session_id=session_id,
                )
                raise ValueError(f"Submission rejected (Cost): {msg}")

            weight_g = validation_result.weight_g
            if constraints.max_weight_g and weight_g > constraints.max_weight_g:
                msg = f"Weight {weight_g:.1f}g exceeds limit {constraints.max_weight_g:.1f}g"
                logger.error(
                    "submission_weight_limit_exceeded",
                    weight=weight_g,
                    limit=constraints.max_weight_g,
                    session_id=session_id,
                )
                raise ValueError(f"Submission rejected (Weight): {msg}")

    # 4. Persist artifacts
    render_paths = []
    logger.info("renders_persisted", count=len(render_paths), session_id=session_id)

    cad_path = renders_dir / "model.step"
    export_step(component, str(cad_path))

    import shutil

    shutil.copy(objectives_path, renders_dir / "objectives.yaml")
    shutil.copy(cost_path, renders_dir / "assembly_definition.yaml")

    # 5. Create reviewer-stage manifest
    stage_to_manifest = {
        "benchmark_reviewer": "benchmark_review_manifest.json",
        "engineering_execution_reviewer": "engineering_execution_review_manifest.json",
        "electronics_reviewer": "electronics_review_manifest.json",
    }
    normalized_stage = (
        reviewer_stage
        if reviewer_stage in stage_to_manifest
        else "engineering_execution_reviewer"
    )
    manifest_name = stage_to_manifest[normalized_stage]
    manifest_path = manifests_dir / manifest_name
    manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage=normalized_stage,
        timestamp=os.getenv("TIMESTAMP"),
        session_id=session_id or os.getenv("SESSION_ID", "default"),
        revision=_latest_git_revision(cwd),
        script_path=str(script_path.relative_to(cwd)),
        script_sha256=script_sha256,
        validation_success=validation_record.success,
        validation_timestamp=validation_record.timestamp,
        simulation_success=simulation_result.success,
        simulation_summary=simulation_result.summary,
        simulation_timestamp=simulation_results_path.stat().st_mtime,
        goal_reached=_goal_reached(simulation_result.summary),
        renders=render_paths,
        mjcf_path=str(renders_dir / "scene.xml"),
        cad_path=str(cad_path),
        objectives_path=str(renders_dir / "objectives.yaml"),
        assembly_definition_path=str(renders_dir / "assembly_definition.yaml"),
    )

    manifest_json = manifest.model_dump_json(indent=2)
    with manifest_path.open("w", encoding="utf-8") as f:
        f.write(manifest_json)

    # Mirror manifest to renders folder for artifact surfacing.
    synced_manifest_path = renders_dir / manifest_name
    with synced_manifest_path.open("w", encoding="utf-8") as f:
        f.write(manifest_json)

    logger.info(
        "handover_complete",
        manifest=str(manifest_path),
        synced_manifest=str(synced_manifest_path),
        session_id=session_id,
    )
    return True
