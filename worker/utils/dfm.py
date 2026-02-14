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


def _count_dofs(part: Part | Compound) -> int:
    """
    Count degrees of freedom in a compound assembly.

    Per architecture spec: Warn if DOF >= 4 as it's unusual in engineering.
    This is a simplified heuristic based on child count of compounds.

    Returns:
        Estimated DOF count based on assembly structure
    """
    if isinstance(part, Part):
        # Single parts typically have 0 internal DOFs
        return 0

    # For compounds, each child that could move independently adds DOFs
    # A simple heuristic: count children that are not zones or fixed
    dof_count = 0
    for child in part.children:
        label = getattr(child, "label", "")
        # Skip zones (they don't contribute to mechanical DOF)
        if label.startswith("zone_"):
            continue
        # Check if part is marked as fixed
        if getattr(child, "fixed", False):
            continue
        # Each free part could add up to 6 DOFs (3 translation + 3 rotation)
        # But we count conservatively as 1 DOF per movable part
        dof_count += 1

    return dof_count


def validate_and_price(
    part: Part | Compound,
    method: ManufacturingMethod,
    config: ManufacturingConfig,
    build_zone: BoundingBox | None = None,
    quantity: int = 1,
    fem_required: bool = False,
) -> WorkbenchResult:
    """
    Unified entry point for DFM (Design for Manufacturing) validation and pricing.
    Dispatches to the appropriate workbench analysis function.

    Args:
        part: The build123d Part or Compound to validate
        method: Manufacturing method (CNC, 3DP, IM)
        config: Manufacturing configuration
        build_zone: Optional build zone bounds to validate against
        quantity: Number of units
        fem_required: If True, validates presence of FEM material fields (WP2/INT-111)

    Returns:
        WorkbenchResult with manufacturability, cost, and violations
    """
    logger.info(
        "starting_dfm_facade_analysis", method=method, fem_required=fem_required
    )

    # First, check build zone if provided
    build_zone_violations: list[str] = []
    if build_zone is not None:
        is_valid, error_msg = _is_within_bounds(part, build_zone)
        if not is_valid:
            build_zone_violations = [f"Build zone violation: {error_msg}"]
            logger.warning("build_zone_violations", violations=build_zone_violations)

    # Check for excessive DOFs (per architecture spec Item 8)
    dof_count = _count_dofs(part)
    dof_warning = dof_count >= 4
    if dof_warning:
        logger.warning(
            "dof_warning",
            dof_count=dof_count,
            message=f"Compound has {dof_count} DOFs - unusual in engineering",
        )

    # Dispatch to appropriate workbench
    if method == ManufacturingMethod.CNC:
        result = analyze_cnc(part, config, quantity=quantity)
    elif method == ManufacturingMethod.INJECTION_MOLDING:
        result = analyze_im(part, config, quantity=quantity)
    elif method == ManufacturingMethod.THREE_DP:
        result = analyze_3dp(part, config, quantity=quantity)
    else:
        logger.error("unsupported_manufacturing_method", method=method)
        raise ValueError(f"Unsupported manufacturing method: {method}")

    # WP2: FEM Material Field Validation (INT-111)
    fem_violations: list[str] = []
    if fem_required:
        material_id = getattr(part, "metadata", {}).get("material_id")
        # Check global materials and method-specific materials
        mat_def = config.materials.get(material_id)
        if not mat_def and method == ManufacturingMethod.CNC and config.cnc:
            mat_def = config.cnc.materials.get(material_id)
        elif (
            not mat_def
            and method == ManufacturingMethod.INJECTION_MOLDING
            and config.injection_molding
        ):
            mat_def = config.injection_molding.materials.get(material_id)
        elif not mat_def and method == ManufacturingMethod.THREE_DP and config.three_dp:
            mat_def = config.three_dp.materials.get(material_id)

        if not mat_def:
            fem_violations = [
                f"FEM Validation Error: Material '{material_id}' not found in configuration."
            ]
        else:
            required_fields = [
                "youngs_modulus_pa",
                "poissons_ratio",
                "yield_stress_pa",
                "ultimate_stress_pa",
            ]
            missing = [f for f in required_fields if getattr(mat_def, f) is None]
            if missing:
                fem_violations = [
                    f"FEM Validation Error: Material '{material_id}' missing required FEM fields: {', '.join(missing)}"
                ]

    # Add DOF warning to metadata (for reviewer notification)
    result_metadata = dict(result.metadata)
    result_metadata["dof_count"] = dof_count
    result_metadata["dof_warning"] = dof_warning

    # Combine all violations
    all_violations = list(result.violations) + build_zone_violations + fem_violations
    is_manufacturable = (
        result.is_manufacturable and not build_zone_violations and not fem_violations
    )

    return WorkbenchResult(
        is_manufacturable=is_manufacturable,
        unit_cost=result.unit_cost,
        weight_g=result.weight_g,
        violations=all_violations,
        metadata=result_metadata,
    )
