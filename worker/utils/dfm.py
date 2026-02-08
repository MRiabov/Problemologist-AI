import structlog
from build123d import Compound, Part

from worker.workbenches.cnc import analyze_cnc
from worker.workbenches.injection_molding import analyze_im
from worker.workbenches.print_3d import analyze_3dp
from worker.workbenches.models import (
    ManufacturingConfig,
    ManufacturingMethod,
    WorkbenchResult,
)

logger = structlog.get_logger()


def validate_and_price(
    part: Part | Compound, method: ManufacturingMethod, config: ManufacturingConfig
) -> WorkbenchResult:
    """
    Unified entry point for DFM (Design for Manufacturing) validation and pricing.
    Dispatches to the appropriate workbench analysis function.
    """
    logger.info("starting_dfm_facade_analysis", method=method)

    if method == ManufacturingMethod.CNC:
        return analyze_cnc(part, config)
    if method == ManufacturingMethod.INJECTION_MOLDING:
        return analyze_im(part, config)
    if method == ManufacturingMethod.THREE_DP:
        return analyze_3dp(part, config)
    logger.error("unsupported_manufacturing_method", method=method)
    raise ValueError(f"Unsupported manufacturing method: {method}")
