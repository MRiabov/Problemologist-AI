import os
from hashlib import sha256
from pathlib import Path

import structlog
from build123d import Compound, export_step

from shared.git_utils import repo_revision
from shared.models.simulation import SimulationResult
from shared.workers.schema import (
    RenderManifest,
    ReviewerStage,
    ReviewManifest,
    ValidationResultRecord,
)
from worker_heavy.utils.dfm import validate_and_price_assembly
from worker_heavy.utils.file_validation import (
    validate_declared_planner_cost_contract,
    validate_environment_attachment_contract,
)
from worker_heavy.workbenches.config import load_required_merged_config

logger = structlog.get_logger(__name__)


def _sha256_file(path: Path) -> str:
    return sha256(path.read_bytes()).hexdigest()


def _goal_reached(summary: str) -> bool:
    s = (summary or "").lower()
    return "goal achieved" in s or "green zone" in s or "goal zone" in s


def _is_static_preview_render(path: str) -> bool:
    return Path(path).suffix.lower() in {".png", ".jpg", ".jpeg"}


def _normalize_render_path(path: str) -> str:
    normalized = str(Path(path))
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    return normalized


def _validate_render_manifest_bundle(
    *, renders_dir: Path, render_paths: list[str]
) -> None:
    expected_render_paths = {
        _normalize_render_path(path)
        for path in render_paths
        if _is_static_preview_render(path)
    }
    if not expected_render_paths:
        return

    manifest_path = renders_dir / "render_manifest.json"
    if not manifest_path.exists():
        missing_render_files = sorted(
            path
            for path in expected_render_paths
            if not (renders_dir.parent / path.lstrip("/")).exists()
        )
        if missing_render_files:
            raise ValueError(
                f"latest preview bundle is missing render files: {missing_render_files}"
            )
        raise ValueError(
            "renders/render_manifest.json missing for latest preview bundle"
        )

    try:
        render_manifest = RenderManifest.model_validate_json(
            manifest_path.read_text(encoding="utf-8")
        )
    except Exception as exc:
        raise ValueError(f"renders/render_manifest.json is invalid: {exc}") from exc

    actual_render_paths = {
        _normalize_render_path(path)
        for path in render_manifest.artifacts.keys()
        if _is_static_preview_render(path)
    }
    if actual_render_paths == expected_render_paths:
        return

    missing = sorted(expected_render_paths - actual_render_paths)
    unexpected = sorted(actual_render_paths - expected_render_paths)
    details: list[str] = []
    if missing:
        details.append(f"missing entries: {missing}")
    if unexpected:
        details.append(f"unexpected entries: {unexpected}")
    raise ValueError(
        "renders/render_manifest.json is out of sync with the latest preview bundle: "
        + "; ".join(details)
    )


def _latest_git_revision(cwd: Path) -> str | None:
    return repo_revision(cwd) or repo_revision(Path(__file__).resolve().parents[2])


def _resolve_submission_assembly_definition(
    cwd: Path, reviewer_stage: ReviewerStage
) -> tuple[Path, str]:
    """Pick the stage-correct assembly definition artifact for review handover."""
    if reviewer_stage == "benchmark_reviewer":
        rel_path = "benchmark_assembly_definition.yaml"
    else:
        rel_path = "assembly_definition.yaml"
    return cwd / rel_path, rel_path


def submit_for_review(
    component: Compound,
    cwd: Path = Path(),
    session_id: str | None = None,
    reviewer_stage: ReviewerStage = "engineering_execution_reviewer",
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
    normalized_stage = reviewer_stage
    allowed_stages = {
        "benchmark_reviewer",
        "engineering_execution_reviewer",
        "electronics_reviewer",
    }
    if normalized_stage not in allowed_stages:
        raise ValueError(f"Unsupported reviewer_stage: {reviewer_stage}")

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
            logger.warning(
                "plan_md_invalid",
                plan_type=plan_type,
                violations=errors,
                session_id=session_id,
            )
            raise ValueError(f"plan.md invalid: {errors}")
    else:
        logger.warning("plan_md_missing", session_id=session_id)
        raise ValueError("plan.md is missing (required for submission)")

    # todo.md
    todo_path = cwd / "todo.md"
    if todo_path.exists():
        from shared.workers.markdown_validator import validate_todo_md

        todo_content = todo_path.read_text(encoding="utf-8")
        todo_result = validate_todo_md(todo_content, require_completion=True)
        if not todo_result.is_valid:
            logger.warning(
                "todo_md_invalid",
                violations=todo_result.violations,
                session_id=session_id,
            )
            raise ValueError(f"todo.md invalid: {todo_result.violations}")
    else:
        logger.warning("todo_md_missing", session_id=session_id)
        raise ValueError("todo.md is missing (required for submission)")

    # benchmark_definition.yaml
    objectives_path = cwd / "benchmark_definition.yaml"
    if objectives_path.exists():
        from .file_validation import validate_benchmark_definition_yaml

        objectives_content = objectives_path.read_text(encoding="utf-8")
        is_valid, result = validate_benchmark_definition_yaml(
            objectives_content, session_id=session_id
        )
        if not is_valid:
            logger.warning(
                "benchmark_definition_yaml_invalid",
                errors=result,
                session_id=session_id,
            )
            raise ValueError(f"benchmark_definition.yaml invalid: {result}")

        # INT-015: Verify immutability
        from .file_validation import validate_immutability

        is_immutable, error_msg = validate_immutability(
            objectives_path, session_id=session_id
        )
        if not is_immutable:
            logger.warning("benchmark_definition_yaml_modified", session_id=session_id)
            raise ValueError(f"benchmark_definition.yaml violation: {error_msg}")
    else:
        logger.warning("benchmark_definition_yaml_missing", session_id=session_id)
        raise ValueError(
            "benchmark_definition.yaml is missing (required for submission)"
        )

    # Stage-specific assembly_definition artifact
    cost_path, assembly_definition_name = _resolve_submission_assembly_definition(
        cwd, normalized_stage
    )
    if cost_path.exists():
        from .file_validation import validate_assembly_definition_yaml

        cost_content = cost_path.read_text(encoding="utf-8")
        custom_config_path = cwd / "manufacturing_config.yaml"
        is_valid, estimation = validate_assembly_definition_yaml(
            cost_content,
            session_id=session_id,
            manufacturing_config=load_required_merged_config(custom_config_path),
        )
        if not is_valid:
            logger.warning(
                f"{cost_path.stem}_yaml_invalid",
                errors=estimation,
                session_id=session_id,
            )
            raise ValueError(f"{assembly_definition_name} invalid: {estimation}")
    else:
        logger.warning(f"{cost_path.stem}_yaml_missing", session_id=session_id)
        raise ValueError(
            f"{assembly_definition_name} is missing (required for submission)"
        )

    # 2. Verify prior validation (INT-018) for current script revision
    script_path = cwd / "script.py"
    if not script_path.exists():
        logger.warning("script_missing_for_handover", session_id=session_id)
        raise ValueError("script.py is missing (required for submission)")
    script_mtime = script_path.stat().st_mtime
    script_sha256 = _sha256_file(script_path)

    validation_results_path = cwd / "validation_results.json"
    if not validation_results_path.exists():
        logger.warning("prior_validation_missing", session_id=session_id)
        raise ValueError(
            "Prior validation missing. Call /benchmark/validate before submission."
        )
    validation_record = ValidationResultRecord.model_validate_json(
        validation_results_path.read_text(encoding="utf-8")
    )
    if not validation_record.success:
        logger.warning("prior_validation_failed", session_id=session_id)
        raise ValueError(
            "Prior validation failed. Fix validation errors before submission."
        )
    if validation_record.script_sha256 != script_sha256:
        logger.warning(
            "prior_validation_stale_for_script",
            session_id=session_id,
            validation_script_sha256=validation_record.script_sha256,
            current_script_sha256=script_sha256,
        )
        raise ValueError(
            "Prior validation is stale for current script revision. Re-run validate."
        )

    # 2b. Verify prior simulation for current script revision and objective success.
    simulation_results_path = cwd / "simulation_result.json"
    if not simulation_results_path.exists():
        logger.warning("prior_simulation_missing", session_id=session_id)
        raise ValueError("Prior simulation missing. Call /benchmark/simulate first.")
    simulation_result = SimulationResult.model_validate_json(
        simulation_results_path.read_text(encoding="utf-8")
    )
    if not simulation_result.success:
        logger.warning(
            "prior_simulation_failed",
            summary=simulation_result.summary,
            session_id=session_id,
        )
        raise ValueError(
            "Prior simulation failed. Submission requires a successful simulation."
        )
    if not _goal_reached(simulation_result.summary):
        logger.warning(
            "goal_not_reached_in_simulation",
            summary=simulation_result.summary,
            session_id=session_id,
        )
        raise ValueError(
            "Simulation did not report goal completion (green/goal zone reached)."
        )
    if simulation_results_path.stat().st_mtime < script_mtime:
        logger.warning("prior_simulation_stale_for_script", session_id=session_id)
        raise ValueError(
            "Prior simulation is stale for current script revision. Re-run simulate."
        )

    # 3. Perform DFM + Geometry Checks (INT-019)
    renders_dir.mkdir(parents=True, exist_ok=True)
    manifests_dir.mkdir(parents=True, exist_ok=True)
    custom_config_path = cwd / "manufacturing_config.yaml"
    dfm_config = load_required_merged_config(custom_config_path)

    objectives_raw = objectives_path.read_text()
    is_valid, objectives_result = validate_benchmark_definition_yaml(
        objectives_raw,
        session_id=session_id,
    )
    if not is_valid:
        raise ValueError(
            "benchmark_definition.yaml invalid: " + "; ".join(objectives_result)
        )
    objectives_model = objectives_result
    attachment_errors = validate_environment_attachment_contract(
        benchmark_definition=objectives_model,
        assembly_definition=estimation,
    )
    if attachment_errors:
        logger.warning(
            "environment_attachment_contract_invalid",
            errors=attachment_errors,
            session_id=session_id,
        )
        raise ValueError(
            "Attachment contract violation: " + "; ".join(attachment_errors)
        )
    cost_errors = validate_declared_planner_cost_contract(
        assembly_definition=estimation,
        manufacturing_config=dfm_config,
    )
    if cost_errors:
        logger.warning(
            "planner_cost_contract_invalid",
            errors=cost_errors,
            session_id=session_id,
        )
        raise ValueError("Pricing contract violation: " + "; ".join(cost_errors))
    build_zone = objectives_model.objectives.build_zone
    constraints = objectives_model.constraints

    manufactured_labels = {part.part_name for part in estimation.manufactured_parts}
    if normalized_stage != "benchmark_reviewer":
        from shared.workers.workbench_models import ManufacturingMethod

        method = (
            estimation.manufactured_parts[0].manufacturing_method
            if estimation.manufactured_parts
            else ManufacturingMethod.CNC
        )
        validation_result = validate_and_price_assembly(
            component,
            dfm_config,
            assembly_definition=estimation,
            part_labels=manufactured_labels or None,
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
                msg = (
                    f"Unit cost ${validation_result.unit_cost:.2f} exceeds limit "
                    f"${constraints.max_unit_cost:.2f}"
                )
                logger.warning(
                    "submission_cost_limit_exceeded",
                    cost=validation_result.unit_cost,
                    limit=constraints.max_unit_cost,
                    session_id=session_id,
                )
                raise ValueError(f"Submission rejected (Cost): {msg}")

            weight_g = validation_result.weight_g
            if constraints.max_weight_g and weight_g > constraints.max_weight_g:
                msg = f"Weight {weight_g:.1f}g exceeds limit {constraints.max_weight_g:.1f}g"
                logger.warning(
                    "submission_weight_limit_exceeded",
                    weight=weight_g,
                    limit=constraints.max_weight_g,
                    session_id=session_id,
                )
                raise ValueError(f"Submission rejected (Weight): {msg}")

    # 4. Persist artifacts
    render_paths: list[str] = []
    for raw_render_path in simulation_result.render_paths:
        src_path = Path(raw_render_path)
        if not src_path.exists() or not src_path.is_file():
            logger.warning(
                "submission_render_missing",
                render_path=str(src_path),
                session_id=session_id,
            )
            continue
        dest_path = renders_dir / src_path.name
        if src_path.resolve() != dest_path.resolve():
            import shutil

            shutil.copy(src_path, dest_path)
        render_paths.append(
            _normalize_render_path(str(Path("renders") / src_path.name))
        )
    _validate_render_manifest_bundle(renders_dir=renders_dir, render_paths=render_paths)
    logger.info("renders_persisted", count=len(render_paths), session_id=session_id)

    cad_path = renders_dir / "model.step"
    export_step(component, str(cad_path))
    if normalized_stage in {
        "benchmark_reviewer",
        "engineering_execution_reviewer",
    }:
        cad_path_value = None
    else:
        cad_path_value = str(cad_path) if cad_path.exists() else None

    import shutil

    shutil.copy(objectives_path, renders_dir / "benchmark_definition.yaml")
    rendered_assembly_definition_path = renders_dir / assembly_definition_name
    shutil.copy(cost_path, rendered_assembly_definition_path)

    revision = _latest_git_revision(cwd)
    if not revision:
        raise ValueError(
            "Unable to determine current repository git revision for review manifest."
        )

    # 5. Create reviewer-stage manifest
    stage_to_manifest = {
        "benchmark_reviewer": "benchmark_review_manifest.json",
        "engineering_execution_reviewer": "engineering_execution_review_manifest.json",
        "electronics_reviewer": "electronics_review_manifest.json",
    }
    manifest_name = stage_to_manifest[normalized_stage]
    manifest_path = manifests_dir / manifest_name
    manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage=normalized_stage,
        timestamp=os.getenv("TIMESTAMP"),
        session_id=session_id or os.getenv("SESSION_ID", "default"),
        revision=revision,
        script_path=str(script_path.relative_to(cwd)),
        script_sha256=script_sha256,
        validation_success=validation_record.success,
        validation_timestamp=validation_record.timestamp,
        simulation_success=simulation_result.success,
        simulation_summary=simulation_result.summary,
        simulation_timestamp=simulation_results_path.stat().st_mtime,
        goal_reached=_goal_reached(simulation_result.summary),
        renders=render_paths,
        mjcf_path=(
            None
            if normalized_stage
            in {"benchmark_reviewer", "engineering_execution_reviewer"}
            else (
                str(renders_dir / "scene.xml")
                if (renders_dir / "scene.xml").exists()
                else None
            )
        ),
        cad_path=cad_path_value,
        objectives_path=(
            None
            if normalized_stage
            in {"benchmark_reviewer", "engineering_execution_reviewer"}
            else str(renders_dir / "benchmark_definition.yaml")
        ),
        assembly_definition_path=(
            None
            if normalized_stage
            in {"benchmark_reviewer", "engineering_execution_reviewer"}
            else str(rendered_assembly_definition_path)
        ),
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
        revision=revision,
        session_id=session_id,
    )
    return True
