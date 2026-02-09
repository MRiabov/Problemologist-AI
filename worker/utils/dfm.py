import structlog
from build123d import Compound, Part

from shared.models.schemas import BoundingBox
from worker.workbenches.cnc import analyze_cnc
from worker.workbenches.injection_molding import analyze_im
from worker.workbenches.models import (
    ManufacturingConfig,
    ManufacturingMethod,
    WorkbenchResult,
)
from worker.workbenches.print_3d import analyze_3dp

logger = structlog.get_logger()


def _is_within_bounds(
    part: Part | Compound, build_zone: BoundingBox
) -> tuple[bool, str]:
    """
    Check if a part's bounding box is fully within the build zone.

    Returns:
        (True, "") if within bounds
        (False, error_message) if out of bounds
    """
    bbox = part.bounding_box()

    # Check each dimension
    violations = []
    if build_zone.min[0] > bbox.min.X:
        violations.append(
            f"X min ({bbox.min.X:.2f}) < build zone min ({build_zone.min[0]:.2f})"
        )
    if build_zone.min[1] > bbox.min.Y:
        violations.append(
            f"Y min ({bbox.min.Y:.2f}) < build zone min ({build_zone.min[1]:.2f})"
        )
    if build_zone.min[2] > bbox.min.Z:
        violations.append(
            f"Z min ({bbox.min.Z:.2f}) < build zone min ({build_zone.min[2]:.2f})"
        )
    if build_zone.max[0] < bbox.max.X:
        violations.append(
            f"X max ({bbox.max.X:.2f}) > build zone max ({build_zone.max[0]:.2f})"
        )
    if build_zone.max[1] < bbox.max.Y:
        violations.append(
            f"Y max ({bbox.max.Y:.2f}) > build zone max ({build_zone.max[1]:.2f})"
        )
    if build_zone.max[2] < bbox.max.Z:
        violations.append(
            f"Z max ({bbox.max.Z:.2f}) > build zone max ({build_zone.max[2]:.2f})"
        )

    if violations:
        return False, "; ".join(violations)
    return True, ""


def validate_and_price(
    part: Part | Compound,
    method: ManufacturingMethod,
    config: ManufacturingConfig,
    build_zone: BoundingBox | None = None,
) -> WorkbenchResult:
    """
    Unified entry point for DFM (Design for Manufacturing) validation and pricing.
    Dispatches to the appropriate workbench analysis function.

    Args:
        part: The build123d Part or Compound to validate
        method: Manufacturing method (CNC, 3DP, IM)
        config: Manufacturing configuration
        build_zone: Optional build zone bounds to validate against

    Returns:
        WorkbenchResult with manufacturability, cost, and violations
    """
    logger.info("starting_dfm_facade_analysis", method=method)

    # First, check build zone if provided
    build_zone_violations: list[str] = []
    if build_zone is not None:
        is_valid, error_msg = _is_within_bounds(part, build_zone)
        if not is_valid:
            build_zone_violations = [f"Build zone violation: {error_msg}"]
            logger.warning("build_zone_violations", violations=build_zone_violations)

    # Dispatch to appropriate workbench
    if method == ManufacturingMethod.CNC:
        result = analyze_cnc(part, config)
    elif method == ManufacturingMethod.INJECTION_MOLDING:
        result = analyze_im(part, config)
    elif method == ManufacturingMethod.THREE_DP:
        result = analyze_3dp(part, config)
    else:
        logger.error("unsupported_manufacturing_method", method=method)
        raise ValueError(f"Unsupported manufacturing method: {method}")

    # Combine build zone violations with workbench violations
    if build_zone_violations:
        all_violations = list(result.violations) + build_zone_violations
        return WorkbenchResult(
            is_manufacturable=False,  # Not manufacturable if outside build zone
            unit_cost=result.unit_cost,
            violations=all_violations,
            metadata=result.metadata,
        )

    return result
