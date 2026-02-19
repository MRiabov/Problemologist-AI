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

logger = structlog.get_logger(__name__)


def submit_for_review(component: Compound, cwd: Path = Path()):
    """
    Standardized handover from Coder to Reviewer.
    Logic:
    - Persist temporary assets to the /renders/ folder.
    - Trigger a LangGraph event or update shared state for the Reviewer node.
    """
    logger.info("handover_started", cwd=str(cwd), files=os.listdir(cwd))

    renders_dir = cwd / os.getenv("RENDERS_DIR", "renders")

    # 1. Validate mandatory base files (INT-005)

    # plan.md
    plan_path = cwd / "plan.md"
    if plan_path.exists():
        from .markdown_validator import validate_plan_md

        plan_content = plan_path.read_text(encoding="utf-8")
        plan_result = validate_plan_md(plan_content)
        if not plan_result.is_valid:
            logger.error("plan_md_invalid", violations=plan_result.violations)
            raise ValueError(f"plan.md invalid: {plan_result.violations}")
    else:
        logger.error("plan_md_missing")
        raise ValueError("plan.md is missing (required for submission)")

    # todo.md
    todo_path = cwd / "todo.md"
    if todo_path.exists():
        from .markdown_validator import validate_todo_md

        todo_content = todo_path.read_text(encoding="utf-8")
        todo_result = validate_todo_md(todo_content, require_completion=True)
        if not todo_result.is_valid:
            logger.error("todo_md_invalid", violations=todo_result.violations)
            raise ValueError(f"todo.md invalid: {todo_result.violations}")
    else:
        logger.error("todo_md_missing")
        raise ValueError("todo.md is missing (required for submission)")

    # objectives.yaml
    objectives_path = cwd / "objectives.yaml"
    if objectives_path.exists():
        from .file_validation import validate_objectives_yaml

        objectives_content = objectives_path.read_text(encoding="utf-8")
        is_valid, result = validate_objectives_yaml(objectives_content)
        if not is_valid:
            logger.error("objectives_yaml_invalid", errors=result)
            raise ValueError(f"objectives.yaml invalid: {result}")

        # INT-015: Verify immutability
        from .file_validation import validate_immutability

        is_immutable, error_msg = validate_immutability(objectives_path)
        if not is_immutable:
            logger.error("objectives_yaml_modified")
            raise ValueError(f"objectives.yaml violation: {error_msg}")
    else:
        logger.error("objectives_yaml_missing")
        raise ValueError("objectives.yaml is missing (required for submission)")

    # assembly_definition.yaml
    cost_path = cwd / "assembly_definition.yaml"
    if cost_path.exists():
        from .file_validation import validate_assembly_definition_yaml

        cost_content = cost_path.read_text(encoding="utf-8")
        is_valid, estimation = validate_assembly_definition_yaml(cost_content)
        if not is_valid:
            logger.error("assembly_definition_yaml_invalid", errors=estimation)
            raise ValueError(f"assembly_definition.yaml invalid: {estimation}")
    else:
        logger.error("assembly_definition_yaml_missing")
        raise ValueError(
            "assembly_definition.yaml is missing (required for submission)"
        )

    # 2. Verify prior validation (INT-018)
    validation_results_path = cwd / "validation_results.json"
    if not validation_results_path.exists():
        logger.error("prior_validation_missing")
        raise ValueError(
            "Prior validation missing. Call /benchmark/validate before submission."
        )

    # 3. Perform DFM + Geometry Checks (INT-019)
    renders_dir.mkdir(parents=True, exist_ok=True)
    dfm_config = load_config()

    objectives_data = yaml.safe_load(objectives_path.read_text())
    objectives_model = ObjectivesYaml(**objectives_data)
    build_zone = objectives_model.objectives.build_zone
    constraints = objectives_model.constraints

    # T016: Extract method from assembly definition to avoid hardcoded CNC
    method = ManufacturingMethod.CNC
    if estimation.manufactured_parts:
        # Use primary method from first part
        raw_method = estimation.manufactured_parts[0].manufacturing_method
        try:
            # Handle common case variations (CNC vs cnc, 3DP vs 3dp)
            method = ManufacturingMethod(raw_method.lower())
        except ValueError:
            logger.warning("invalid_manufacturing_method", method=raw_method)

    validation_result = validate_and_price(
        component, method, dfm_config, build_zone=build_zone
    )

    if not validation_result.is_manufacturable:
        logger.error("submission_dfm_failed", violations=validation_result.violations)
        raise ValueError(f"Submission rejected (DFM): {validation_result.violations}")

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

        # Fix: weight_kg check was broken (using .get() on Pydantic model)
        weight_kg = validation_result.weight_g / 1000.0
        if constraints.max_weight and weight_kg > constraints.max_weight:
            msg = f"Weight {weight_kg:.3f}kg exceeds limit {constraints.max_weight:.3f}kg"
            logger.error(
                "submission_weight_limit_exceeded",
                weight=weight_kg,
                limit=constraints.max_weight,
            )
            raise ValueError(f"Submission rejected (Weight): {msg}")

    # 4. Persist artifacts
    render_paths = []
    logger.info("renders_persisted", count=len(render_paths))

    cad_path = renders_dir / "model.step"
    export_step(component, str(cad_path))

    import shutil

    shutil.copy(objectives_path, renders_dir / "objectives.yaml")
    shutil.copy(cost_path, renders_dir / "assembly_definition.yaml")

    # 5. Create manifest
    manifest_path = renders_dir / "review_manifest.json"
    manifest = {
        "status": "ready_for_review",
        "timestamp": os.getenv("TIMESTAMP"),
        "session_id": os.getenv("SESSION_ID", "default"),
        "renders": render_paths,
        "mjcf_path": str(renders_dir / "scene.xml"),
        "cad_path": str(cad_path),
        "objectives_path": str(renders_dir / "objectives.yaml"),
        "assembly_definition_path": str(renders_dir / "assembly_definition.yaml"),
    }

    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f)

    logger.info("handover_complete", manifest=str(manifest_path))
    return True
