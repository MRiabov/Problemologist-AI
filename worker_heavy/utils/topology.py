from pathlib import Path

import structlog
from build123d import Compound, Part

from shared.workers.workbench_models import ManufacturingMethod, WorkbenchResult
from worker_heavy.utils.dfm import validate_and_price
from worker_heavy.workbenches.config import load_config, load_merged_config

logger = structlog.get_logger(__name__)


def analyze_component(
    component: Part | Compound,
    output_dir: Path,
    method: ManufacturingMethod | None = None,
    quantity: int = 1,
) -> WorkbenchResult:
    """
    Analyzes a component for manufacturability and cost.
    Used by the heavy worker benchmark/analyze endpoint.
    """
    # Prefer a workspace override when present so analyze uses the same merged
    # pricing source as planner validation and handoff submission.
    custom_config_path = output_dir / "manufacturing_config.yaml"
    if custom_config_path.exists():
        config = load_merged_config(custom_config_path)
    else:
        config = load_config()

    if method is None:
        # Heuristic: try to get manufacturing method from component metadata
        # Default to CNC if not specified
        metadata = getattr(component, "metadata", None)
        method_raw = getattr(metadata, "manufacturing_method", ManufacturingMethod.CNC)
        if isinstance(method_raw, ManufacturingMethod):
            method = method_raw
        else:
            try:
                method = ManufacturingMethod(str(method_raw).upper())
            except ValueError:
                logger.error(
                    "invalid_manufacturing_method_in_metadata", method=method_raw
                )
                raise ValueError(
                    f"Unknown manufacturing method in component metadata: {method_raw!r}"
                ) from None

    return validate_and_price(
        component,
        method=method,
        config=config,
        quantity=quantity,
    )
