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
    }

    with open(manifest_path, "w") as f:
        json.dump(manifest, f)

    logger.info("handover_complete", manifest=str(manifest_path))
    return True
