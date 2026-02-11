import json
import os
from pathlib import Path

import structlog
import yaml
from build123d import Compound, export_step

from shared.models.schemas import ObjectivesYaml
from worker.utils.dfm import validate_and_price
from worker.workbenches.config import load_config
from worker.workbenches.models import ManufacturingMethod
from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)


def submit_for_review(component: Compound, cwd: Path = Path(".")):
    """
    Standardized handover from Coder to Reviewer.
    Logic:
    - Persist temporary assets to the /renders/ folder.
    - Trigger a LangGraph event or update shared state for the Reviewer node.
    """
    logger.info("handover_started", cwd=str(cwd))

    renders_dir = cwd / os.getenv("RENDERS_DIR", "renders")

    # Validate plan and todo (strict) before submission when present
    plan_path = cwd / "plan.md"
    if plan_path.exists():
        from .markdown_validator import validate_plan_md

        plan_content = plan_path.read_text(encoding="utf-8")
        plan_result = validate_plan_md(plan_content)
        if not plan_result.is_valid:
            logger.error("plan_md_invalid", violations=plan_result.violations)
            raise ValueError(f"plan.md invalid: {plan_result.violations}")
    else:
        # INT-005: plan.md is mandatory
        logger.error("plan_md_missing")
        raise ValueError("plan.md is missing (required for submission)")

    todo_path = cwd / "todo.md"
    if todo_path.exists():
        from .markdown_validator import validate_todo_md

        todo_content = todo_path.read_text(encoding="utf-8")
        todo_result = validate_todo_md(todo_content, require_completion=True)
        if not todo_result.is_valid:
            logger.error("todo_md_invalid", violations=todo_result.violations)
            raise ValueError(f"todo.md invalid: {todo_result.violations}")
    else:
        # INT-005: todo.md is mandatory
        logger.error("todo_md_missing")
        raise ValueError("todo.md is missing (required for submission)")

    # Validate objectives.yaml (Item 1, 2 of review)
    objectives_path = cwd / "objectives.yaml"
    if objectives_path.exists():
        from .file_validation import validate_objectives_yaml

        objectives_content = objectives_path.read_text(encoding="utf-8")
        is_valid, result = validate_objectives_yaml(objectives_content)
        if not is_valid:
            logger.error("objectives_yaml_invalid", errors=result)
            raise ValueError(f"objectives.yaml invalid: {result}")

    else:
        # INT-005: objectives.yaml is mandatory
        logger.error("objectives_yaml_missing")
        raise ValueError("objectives.yaml is missing (required for submission)")

    # INT-015: Verify immutability of objectives.yaml
    if objectives_path.exists():
        from .file_validation import validate_immutability

        is_immutable, error_msg = validate_immutability(objectives_path)
        if not is_immutable:
            logger.error("objectives_yaml_modified")
            raise ValueError(f"objectives.yaml violation: {error_msg}")

    # Validate preliminary_cost_estimation.yaml (if present or required)
    cost_path = cwd / "preliminary_cost_estimation.yaml"
    if cost_path.exists():
        from .file_validation import validate_preliminary_cost_estimation_yaml

        cost_content = cost_path.read_text(encoding="utf-8")
        is_valid, result = validate_preliminary_cost_estimation_yaml(cost_content)
        if not is_valid:
            logger.error("cost_estimation_yaml_invalid", errors=result)
            raise ValueError(f"preliminary_cost_estimation.yaml invalid: {result}")
    else:
        # Note: architecture says it's required for Planner handover
        logger.error("preliminary_cost_estimation_yaml_missing")
        raise ValueError(
            "preliminary_cost_estimation.yaml is missing (required for submission)"
        )

    # Ensure renders_dir exists
    renders_dir.mkdir(parents=True, exist_ok=True)

    # INT-018 & INT-019: Validate and Price (Gate)
    # Load config and validate
    dfm_config = load_config()

    # Load objectives for constraints
    objectives_data = None
    if objectives_path.exists():
        objectives_data = yaml.safe_load(objectives_path.read_text())
        objectives_model = ObjectivesYaml(**objectives_data)
        build_zone = objectives_model.objectives.build_zone
        constraints = objectives_model.constraints
    else:
        build_zone = None
        constraints = None

    # Perform DFM + Geometry Checks
    # Assuming CNC for now as standard, or should infer? Architecture implies CNC/3DP.
    # We'll use CNC as the baseline metric for cost unless specified otherwise.
    validation_result = validate_and_price(
        component, ManufacturingMethod.CNC, dfm_config, build_zone=build_zone
    )

    if not validation_result.is_manufacturable:
        # INT-019: Logic/DFM Failures are hard blockers?
        # Architecture says: "Submit fails if... geometry is invalid or unmanufacturable"
        logger.error("submission_dfm_failed", violations=validation_result.violations)
        raise ValueError(f"Submission rejected (DFM): {validation_result.violations}")

    # Check constraints (INT-019)
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
            )
            raise ValueError(f"Submission rejected (Cost): {msg}")

        if (
            constraints.max_weight
            and validation_result.metadata.get("weight_kg", 0) > constraints.max_weight
        ):
            weight = validation_result.metadata.get("weight_kg", 0)
            msg = f"Weight {weight:.3f}kg exceeds limit {constraints.max_weight:.3f}kg"
            logger.error(
                "submission_weight_limit_exceeded",
                weight=weight,
                limit=constraints.max_weight,
            )
            raise ValueError(f"Submission rejected (Weight): {msg}")

    # 1. Persist renders
    # Generate preview artifacts (DISABLED for integration tests/headless stability)
    # from .rendering import prerender_24_views
    # render_paths = prerender_24_views(component, renders_dir)
    render_paths = []  # Placeholder since prerender_24_views is commented out
    logger.info("renders_persisted", count=len(render_paths))

    # 2. Save models
    cad_path = renders_dir / "model.step"
    export_step(component, str(cad_path))

    # 3. Copy objectives.yaml if it exists
    objectives_path = cwd / "objectives.yaml"
    target_objectives_path = renders_dir / "objectives.yaml"
    if objectives_path.exists():
        import shutil

        shutil.copy(objectives_path, target_objectives_path)
        logger.info("objectives_yaml_persisted")

    # 4. Copy preliminary_cost_estimation.yaml if it exists
    target_cost_path = renders_dir / "preliminary_cost_estimation.yaml"
    if cost_path.exists():
        shutil.copy(cost_path, target_cost_path)
        logger.info("cost_estimation_yaml_persisted")

    # 4. Create review manifest (signal for next node)
    manifest_path = renders_dir / "review_manifest.json"
    manifest = {
        "status": "ready_for_review",
        "timestamp": os.getenv("TIMESTAMP"),
        "session_id": os.getenv("SESSION_ID", "default"),
        "renders": render_paths,
        "mjcf_path": str(renders_dir / "scene.xml"),  # Created by simulate()
        "cad_path": str(cad_path),
        "objectives_path": str(target_objectives_path)
        if target_objectives_path.exists()
        else None,
        "preliminary_cost_estimation_path": str(target_cost_path)
        if target_cost_path.exists()
        else None,
    }

    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f)

    logger.info("handover_complete", manifest=str(manifest_path))
    return True
