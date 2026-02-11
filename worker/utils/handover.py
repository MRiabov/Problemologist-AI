import json
import os
from pathlib import Path

import structlog
from build123d import Compound, export_step

from .rendering import prerender_24_views

logger = structlog.get_logger(__name__)


def submit_for_review(component: Compound):
    """
    Standardized handover from Coder to Reviewer.
    Logic:
    - Persist temporary assets to the /renders/ folder.
    - Trigger a LangGraph event or update shared state for the Reviewer node.
    """
    logger.info("handover_started")

    renders_dir = Path(os.getenv("RENDERS_DIR", "./renders"))

    # Validate plan and todo (strict) before submission when present
    plan_path = Path("plan.md")
    if plan_path.exists():
        from .markdown_validator import validate_plan_md

        plan_content = plan_path.read_text(encoding="utf-8")
        plan_result = validate_plan_md(plan_content)
        if not plan_result.is_valid:
            logger.error("plan_md_invalid", violations=plan_result.violations)
            raise ValueError(f"plan.md invalid: {plan_result.violations}")

    todo_path = Path("todo.md")
    if todo_path.exists():
        from .markdown_validator import validate_todo_md

        todo_content = todo_path.read_text(encoding="utf-8")
        todo_result = validate_todo_md(todo_content, require_completion=True)
        if not todo_result.is_valid:
            logger.error("todo_md_invalid", violations=todo_result.violations)
            raise ValueError(f"todo.md invalid: {todo_result.violations}")
    else:
        logger.warning("todo_md_missing")

    # Validate objectives.yaml (Item 1, 2 of review)
    objectives_path = Path("objectives.yaml")
    if objectives_path.exists():
        from .file_validation import validate_objectives_yaml

        objectives_content = objectives_path.read_text(encoding="utf-8")
        is_valid, result = validate_objectives_yaml(objectives_content)
        if not is_valid:
            logger.error("objectives_yaml_invalid", errors=result)
            raise ValueError(f"objectives.yaml invalid: {result}")
    else:
        logger.warning("objectives_yaml_missing")

    # Validate preliminary_cost_estimation.yaml (if present or required)
    cost_path = Path("preliminary_cost_estimation.yaml")
    if cost_path.exists():
        from .file_validation import validate_preliminary_cost_estimation_yaml

        cost_content = cost_path.read_text(encoding="utf-8")
        is_valid, result = validate_preliminary_cost_estimation_yaml(cost_content)
        if not is_valid:
            logger.error("cost_estimation_yaml_invalid", errors=result)
            raise ValueError(f"preliminary_cost_estimation.yaml invalid: {result}")
    else:
        # Note: architecture says it's required for Planner handover
        logger.warning("preliminary_cost_estimation_yaml_missing")

    # Ensure renders_dir exists
    renders_dir.mkdir(parents=True, exist_ok=True)

    # 1. Persist renders
    render_paths = prerender_24_views(component)
    logger.info("renders_persisted", count=len(render_paths))

    # 2. Save models
    cad_path = renders_dir / "model.step"
    export_step(component, str(cad_path))

    # 3. Copy objectives.yaml if it exists
    objectives_path = Path("objectives.yaml")
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
