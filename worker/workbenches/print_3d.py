from typing import Any

import structlog
from build123d import Compound, Part, Solid

from shared.type_checking import type_check
from worker.workbenches.analysis_utils import (
    check_wall_thickness,
    compute_part_hash,
    part_to_trimesh,
)
from worker.workbenches.base import Workbench
from worker.workbenches.models import (
    CostBreakdown,
    ManufacturingConfig,
    WorkbenchResult,
)

logger = structlog.get_logger()


@type_check
def calculate_3dp_cost(
    part: Part | Compound | Solid,
    config: ManufacturingConfig,
    quantity: int = 1,
    context: dict[str, Any] | None = None,
) -> CostBreakdown:
    """
    Calculates 3D Printing cost: Setup + (Material + Run) * Quantity.
    """
    three_dp_cfg = config.three_dp
    if not three_dp_cfg:
        raise ValueError("3D Printing configuration missing")

    # Default to abs for now
    material_name = config.defaults.get("material", "abs")
    if material_name not in three_dp_cfg.materials:
        material_name = list(three_dp_cfg.materials.keys())[0]

    material_cfg = three_dp_cfg.materials[material_name]

    logger.info("calculating_3dp_cost", material=material_name, quantity=quantity)

    # 1. Material Cost
    # Volume in cm3 (build123d volume is in mm3)
    volume_cm3 = part.volume / 1000.0
    density = material_cfg.density_g_cm3
    cost_per_kg = material_cfg.cost_per_kg

    mass_kg = (volume_cm3 * density) / 1000.0
    material_cost_per_part = mass_kg * cost_per_kg

    # 2. Run Cost (Machine Time)
    # Estimate printing time based on volume and deposition rate
    # Default deposition rate: 15 cm3/hr if not specified
    deposition_rate_cm3_hr = three_dp_cfg.parameters.get("deposition_rate_cm3_hr", 15.0)
    printing_time_hr = volume_cm3 / deposition_rate_cm3_hr

    machine_hourly_rate = material_cfg.machine_hourly_rate
    run_cost_per_part = printing_time_hr * machine_hourly_rate

    # 3. Setup Cost
    setup_cost = three_dp_cfg.costs.get("setup_fee", 10.0)

    # Apply reuse discount if part hash is in context
    is_reused = False
    if context is not None:
        part_hash = compute_part_hash(part)
        if part_hash in context:
            setup_cost *= 0.5  # 50% discount on setup for repeated parts
            is_reused = True
        context[part_hash] = context.get(part_hash, 0) + quantity

    unit_cost = material_cost_per_part + run_cost_per_part
    total_cost = setup_cost + (unit_cost * quantity)

    return CostBreakdown(
        process="print_3d",
        total_cost=total_cost,
        unit_cost=total_cost / quantity if quantity > 0 else 0.0,
        material_cost_per_unit=round(material_cost_per_part, 4),
        setup_cost=round(setup_cost, 2),
        is_reused=is_reused,
        details={
            "part_volume_cm3": round(volume_cm3, 2),
            "printing_time_hr": round(printing_time_hr, 2),
            "run_cost_per_unit": round(run_cost_per_part, 4),
        },
        pricing_explanation=(
            f"3DP cost (${total_cost:.2f}) for {quantity} units. "
            f"Material: {material_name} (${material_cost_per_part:.4f}/unit). "
            f"Print time: {printing_time_hr:.2f} hr. "
            f"Setup fee: ${setup_cost:.2f}."
        ),
    )


@type_check
def analyze_3dp(
    part: Part | Compound | Solid, config: ManufacturingConfig, quantity: int = 1
) -> WorkbenchResult:
    """
    Functional entry point for 3D Printing analysis.
    """
    logger.info("starting_3dp_analysis")

    violations = []

    # 1. Geometric Validity Check
    if not part.is_valid:
        violations.append("Geometry is not valid (non-manifold or self-intersecting)")

    # 2. Closed Geometry Check
    solids = part.solids()
    if not solids:
        violations.append("Geometry contains no solids")
    else:
        for i, solid in enumerate(solids):
            # Check for watertightness
            if hasattr(solid, "is_closed") and not solid.is_closed:
                violations.append(f"Solid {i} is not closed (not watertight)")

    # 3. Single Body Check
    if len(solids) > 1:
        violations.append(f"Geometry must be a single body, found {len(solids)} solids")

    # 4. Wall Thickness Check
    three_dp_cfg = config.three_dp
    constraints = three_dp_cfg.constraints if three_dp_cfg else {}
    min_wall = constraints.get("min_wall_thickness_mm", 0.8)
    max_wall = constraints.get("max_wall_thickness_mm", 5.0)

    try:
        mesh = part_to_trimesh(part)
        violations.extend(check_wall_thickness(mesh, min_wall, max_wall))
    except Exception as e:
        logger.warning("wall_thickness_check_failed", error=str(e))

    # 5. Cost Calculation
    # We proceed with cost calculation even if there are violations, unless critical?
    # Usually cost is only valid if manufacturable, but giving an estimate is sometimes useful.
    # However, if it's not a valid solid, volume might be wrong.
    try:
        cost_breakdown = calculate_3dp_cost(part, config, quantity=quantity)
        unit_cost = cost_breakdown.unit_cost
    except Exception as e:
        logger.error("3dp_cost_calculation_failed", error=str(e))
        unit_cost = 0.0
        cost_breakdown = None

    # 6. Weight Calculation
    material_name = config.defaults.get("material", "abs")
    three_dp_cfg = config.three_dp
    density = 1.04  # fallback (ABS)
    if three_dp_cfg and material_name in three_dp_cfg.materials:
        density = three_dp_cfg.materials[material_name].density_g_cm3

    weight_g = (part.volume / 1000.0) * density

    is_manufacturable = len(violations) == 0

    logger.info(
        "3dp_analysis_complete",
        is_manufacturable=is_manufacturable,
        violations=len(violations),
    )

    metadata = {}
    if cost_breakdown:
        metadata["cost_breakdown"] = cost_breakdown.model_dump()

    return WorkbenchResult(
        is_manufacturable=is_manufacturable,
        unit_cost=unit_cost,
        weight_g=weight_g,
        violations=violations,
        metadata=metadata,
    )


@type_check
class Print3DWorkbench(Workbench):
    """
    Workbench for 3D Printing (FDM/SLA).
    Enforces manifold geometry and single-body parts.
    """

    def __init__(self, config: ManufacturingConfig | None = None):
        from worker.workbenches.config import load_config

        self.config = config or load_config()

    def validate(self, part: Part) -> list[Exception | str]:
        result = analyze_3dp(part, self.config)
        return result.violations

    def calculate_cost(
        self,
        part: Part,
        quantity: int = 1,
        context: dict[str, Any] | None = None,
    ) -> CostBreakdown:
        return calculate_3dp_cost(part, self.config, quantity, context)
