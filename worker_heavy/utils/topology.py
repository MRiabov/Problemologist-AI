import structlog
from pathlib import Path
from build123d import Part, Compound

from shared.workers.workbench_models import WorkbenchResult, ManufacturingMethod
from worker_heavy.utils.dfm import validate_and_price
from worker_heavy.workbenches.config import load_config

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
    # Load default manufacturing config
    config = load_config()

    if method is None:
        # Heuristic: try to get manufacturing method from component metadata
        # Default to CNC if not specified
        metadata = getattr(component, "metadata", None)
        method_str = getattr(metadata, "manufacturing_method", "cnc")
        try:
            method = ManufacturingMethod(method_str.lower())
        except ValueError:
            logger.warning("invalid_manufacturing_method_in_metadata", method=method_str)
            method = ManufacturingMethod.CNC

    return validate_and_price(
        component,
        method=method,
        config=config,
        quantity=quantity,
    )
