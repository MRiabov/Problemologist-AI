import hashlib
from typing import Any

import structlog
from build123d import Compound, Part

from shared.type_checking import type_check
from worker.workbenches.analysis_utils import compute_part_hash
from worker.workbenches.base import Workbench
from worker.workbenches.models import (
    CostBreakdown,
    ManufacturingConfig,
    WorkbenchResult,
)

logger = structlog.get_logger()


@type_check
def calculate_3dp_cost(
    part: Part | Compound,
    config: ManufacturingConfig,
    quantity: int = 1,
    context: dict[str, Any] | None = None,
) -> CostBreakdown:
    """
    Calculates 3D printing cost: Setup + (Material + Machine Time) * Quantity.
    """
    tdp_cfg = config.three_dp
    if not tdp_cfg:
        raise ValueError("3D Printing configuration missing")

    # Default to ABS if not specified
    material_name = config.defaults.get("material", "abs")
    if material_name not in tdp_cfg.materials:
        material_name = list(tdp_cfg.materials.keys())[0]

    material_cfg = tdp_cfg.materials[material_name]
    costs_cfg = tdp_cfg.costs

    logger.info("calculating_3dp_cost", material=material_name, quantity=quantity)

    # 1. Material Cost
    volume_cm3 = part.volume / 1000.0
    density = material_cfg.get("density_g_cm3", 1.04)
    cost_per_kg = material_cfg.get("cost_per_kg", 20.0)
    mass_kg = (volume_cm3 * density) / 1000.0
    material_cost_per_part = mass_kg * cost_per_kg

    # 2. Machine Time Cost
    # Simple heuristic: print speed in cm3/hr
    # If not in config, assume 15 cm3/hr (reasonable for FDM)
    print_speed_cm3_hr = tdp_cfg.parameters.get("print_speed_cm3_hr", 15.0)
    print_time_hr = volume_cm3 / print_speed_cm3_hr

    machine_hourly_rate = material_cfg.get("machine_hourly_rate", 5.0)
    machine_cost_per_part = print_time_hr * machine_hourly_rate

    unit_cost = material_cost_per_part + machine_cost_per_part

    # 3. Setup Cost
    setup_cost = costs_cfg.get("setup_fee", 10.0)

    # Apply reuse discount if part hash is in context
    is_reused = False
    if context is not None:
        part_hash = compute_part_hash(part)
        if part_hash in context:
            setup_cost *= 0.5  # 50% discount on setup for repeated runs
            is_reused = True
        context[part_hash] = context.get(part_hash, 0) + quantity

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
            "print_time_hr": round(print_time_hr, 2),
            "material_cost_per_unit": round(material_cost_per_part, 4),
            "machine_cost_per_unit": round(machine_cost_per_part, 4),
        },
        pricing_explanation=(
            f"3D Print cost (${total_cost:.2f}) for {quantity} units. "
            f"Material: {material_name} (${material_cost_per_part:.2f}/unit). "
            f"Print time: {print_time_hr:.2f} hr. "
            f"Setup fee: ${setup_cost:.2f}."
        ),
    )


@type_check
def analyze_3dp(part: Part | Compound, config: ManufacturingConfig) -> WorkbenchResult:
    """
    Functional entry point for 3D Printing analysis.
    """
    logger.info("starting_3dp_analysis")

    violations = []

    # 1. Geometric Validity (manifold, no self-intersections)
    if not part.is_valid:
        violations.append("Geometry is not valid (non-manifold or self-intersecting)")

    # 2. Watertight Check (Closed Geometry)
    solids = part.solids()
    if not solids:
        violations.append("Geometry contains no solids")
    else:
        for i, solid in enumerate(solids):
            if hasattr(solid, "is_closed") and not solid.is_closed:
                violations.append(f"Solid {i} is not closed (not watertight)")

    # 3. Single Body Check
    if len(solids) > 1:
        violations.append(
            f"Geometry must be a single body, found {len(solids)} solids"
        )

    # 4. Cost Calculation (single unit)
    cost_breakdown = calculate_3dp_cost(part, config, quantity=1)

    is_manufacturable = len(violations) == 0

    logger.info(
        "3dp_analysis_complete",
        is_manufacturable=is_manufacturable,
        violations=len(violations),
    )

    return WorkbenchResult(
        is_manufacturable=is_manufacturable,
        unit_cost=cost_breakdown.unit_cost,
        violations=violations,
        metadata={
            "cost_breakdown": cost_breakdown.model_dump(),
        },
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
