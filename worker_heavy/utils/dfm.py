import structlog
from build123d import Compound, Part
from pathlib import Path
from typing import Any

from shared.models.schemas import AssemblyDefinition, BoundingBox
from shared.workers.workbench_models import (
    ManufacturingConfig,
    ManufacturingMethod,
    WorkbenchMetadata,
    WorkbenchResult,
)
from worker_heavy.workbenches.cnc import analyze_cnc
from worker_heavy.workbenches.config import load_required_merged_config
from worker_heavy.workbenches.injection_molding import analyze_im
from worker_heavy.workbenches.print_3d import analyze_3dp

logger = structlog.get_logger()


def load_planner_manufacturing_config(
    config_path: str | Path | None = None,
    override_data: dict[str, Any] | None = None,
) -> ManufacturingConfig:
    """Load the planner's merged manufacturing config with fail-closed semantics."""
    return load_required_merged_config(
        config_path=config_path,
        override_data=override_data,
        source_name="manufacturing_config.yaml",
    )


def _part_reports_for_analysis(part: Part | Compound) -> list[Part | Compound]:
    children = getattr(part, "children", [])
    if not children:
        return [part]
    return [
        child
        for child in children
        if not getattr(child, "label", "").startswith("zone_")
    ]


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


def calculate_benchmark_drilling_cost(
    assembly_definition: AssemblyDefinition,
    config: ManufacturingConfig,
) -> float:
    """Return static benchmark drilling cost from declared environment operations."""
    unit_cost = config.benchmark_operations.drilling.cost_per_hole_usd
    total_holes = sum(
        operation.quantity
        for operation in assembly_definition.environment_drill_operations
    )
    return round(total_holes * unit_cost, 2)


def calculate_declared_assembly_cost(
    assembly_definition: AssemblyDefinition,
    config: ManufacturingConfig,
) -> float:
    """Return deterministic planner-declared assembly cost minimum."""
    manufactured_cost = sum(
        part.estimated_unit_cost_usd * part.quantity
        for part in assembly_definition.manufactured_parts
    )
    cots_cost = sum(
        part.unit_cost_usd * part.quantity for part in assembly_definition.cots_parts
    )
    drilling_cost = calculate_benchmark_drilling_cost(assembly_definition, config)
    return round(manufactured_cost + cots_cost + drilling_cost, 2)


def validate_declared_assembly_cost(
    assembly_definition: AssemblyDefinition,
    config: ManufacturingConfig,
) -> list[str]:
    """Ensure planner totals include all declared part, COTS, and drilling costs."""
    minimum_cost = calculate_declared_assembly_cost(assembly_definition, config)
    if assembly_definition.totals.estimated_unit_cost_usd + 1e-6 < minimum_cost:
        return [
            "assembly_definition.totals.estimated_unit_cost_usd "
            f"(${assembly_definition.totals.estimated_unit_cost_usd:.2f}) "
            "must include declared manufactured-part costs, COTS costs, and "
            f"benchmark drilling cost (minimum ${minimum_cost:.2f})"
        ]
    return []


def validate_exact_declared_assembly_cost(
    assembly_definition: AssemblyDefinition,
    config: ManufacturingConfig,
) -> list[str]:
    """Ensure planner totals exactly match the deterministic declared cost."""
    expected_cost = calculate_declared_assembly_cost(assembly_definition, config)
    actual_cost = round(assembly_definition.totals.estimated_unit_cost_usd, 2)
    if actual_cost != expected_cost:
        return [
            "assembly_definition.totals.estimated_unit_cost_usd "
            f"(${actual_cost:.2f}) must equal the deterministic declared cost "
            f"(${expected_cost:.2f})"
        ]
    return []


def validate_and_price(
    part: Part | Compound,
    method: ManufacturingMethod,
    config: ManufacturingConfig,
    build_zone: BoundingBox | None = None,
    quantity: int = 1,
    fem_required: bool = False,
    session_id: str | None = None,
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
        logger.error(
            "unsupported_manufacturing_method", method=method, session_id=session_id
        )
        raise ValueError(f"Unsupported manufacturing method: {method}")

    # WP2: FEM Material Field Validation (INT-111)
    fem_violations: list[str] = []
    if fem_required:
        metadata = getattr(part, "metadata", None)
        material_id = getattr(metadata, "material_id", None)
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

    result_metadata = result.metadata.model_copy(
        update={
            "dof_count": dof_count,
            "dof_warning": dof_warning,
        }
    )

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


def validate_and_price_assembly(
    part: Part | Compound,
    config: ManufacturingConfig,
    assembly_definition: AssemblyDefinition | None = None,
    part_labels: set[str] | None = None,
    build_zone: BoundingBox | None = None,
    quantity: int = 1,
    fem_required: bool = False,
    session_id: str | None = None,
    default_method: ManufacturingMethod = ManufacturingMethod.CNC,
) -> WorkbenchResult:
    """
    Validate compound assemblies per child instead of as one fused stock block.

    Benchmark/environment compounds usually represent separate manufactured parts.
    Treating the whole assembly as one CNC stock body creates false undercut and
    corner violations across disconnected children.
    """
    reports = _part_reports_for_analysis(part)
    if part_labels is not None:
        reports = [
            child
            for child in reports
            if (getattr(child, "label", None) or "") in part_labels
        ]
    if not reports:
        return WorkbenchResult(
            is_manufacturable=True,
            unit_cost=0.0,
            weight_g=0.0,
            violations=[],
            metadata=WorkbenchMetadata(
                additional_info={
                    "part_reports": [],
                    "part_count": 0,
                }
            ),
        )
    if len(reports) == 1 and reports[0] is part:
        metadata = getattr(part, "metadata", None)
        method = getattr(metadata, "manufacturing_method", None) or default_method
        if isinstance(method, str):
            method = ManufacturingMethod(method)
        result = validate_and_price(
            part,
            method,
            config,
            build_zone=build_zone,
            quantity=quantity,
            fem_required=fem_required,
            session_id=session_id,
        )
        if assembly_definition is None:
            return result

        drilling_cost = calculate_benchmark_drilling_cost(assembly_definition, config)
        return WorkbenchResult(
            is_manufacturable=result.is_manufacturable,
            unit_cost=result.unit_cost + drilling_cost,
            weight_g=result.weight_g,
            violations=list(result.violations),
            metadata=result.metadata,
        )

    total_cost = 0.0
    total_weight = 0.0
    violations: list[str] = []
    per_part: list[dict[str, object]] = []
    overall_ok = True

    for child in reports:
        label = getattr(child, "label", None) or "unnamed_part"
        metadata = getattr(child, "metadata", None)
        method = getattr(metadata, "manufacturing_method", None) or default_method
        if isinstance(method, str):
            method = ManufacturingMethod(method)

        child_result = validate_and_price(
            child,
            method,
            config,
            build_zone=build_zone,
            quantity=quantity,
            fem_required=fem_required,
            session_id=session_id,
        )
        total_cost += child_result.unit_cost
        total_weight += child_result.weight_g
        overall_ok = overall_ok and child_result.is_manufacturable
        per_part.append(
            {
                "label": label,
                "method": method.value,
                "is_manufacturable": child_result.is_manufacturable,
                "unit_cost": child_result.unit_cost,
                "weight_g": child_result.weight_g,
            }
        )
        violations.extend(
            f"{label}: {violation}" for violation in child_result.violations
        )

    benchmark_drilling_cost = (
        calculate_benchmark_drilling_cost(assembly_definition, config)
        if assembly_definition is not None
        else 0.0
    )
    total_cost += benchmark_drilling_cost

    return WorkbenchResult(
        is_manufacturable=overall_ok,
        unit_cost=total_cost,
        weight_g=total_weight,
        violations=violations,
        metadata=WorkbenchMetadata(
            additional_info={
                "part_reports": per_part,
                "part_count": len(reports),
                "benchmark_drilling_cost_usd": benchmark_drilling_cost,
            }
        ),
    )
