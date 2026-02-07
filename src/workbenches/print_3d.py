import structlog
from typing import Any

from build123d import Part, Compound

from src.shared.type_checking import type_check
from src.workbenches.analysis_utils import compute_part_hash
from src.workbenches.base import Workbench
from src.workbenches.models import (
    CostBreakdown,
    ManufacturingConfig,
    WorkbenchResult,
)

logger = structlog.get_logger()


@type_check
def _calculate_3dp_cost(
    part: Part | Compound,
    config: ManufacturingConfig,
    quantity: int = 1,
    context: dict[str, Any] | None = None,
) -> CostBreakdown:
    """
    Calculates 3D Printing cost: Setup + (Material + Machine Time) * Quantity.
    """
    threedp_cfg = config.three_dp
    if not threedp_cfg:
        raise ValueError("3D Printing configuration missing")

    # Default to ABS for now if not specified or if the default material is not in 3dp config
    material_name = config.defaults.get("material", "abs")
    if material_name not in threedp_cfg.materials:
        material_name = list(threedp_cfg.materials.keys())[0]

    material_cfg = threedp_cfg.materials[material_name]

    logger.info("calculating_3dp_cost", material=material_name, quantity=quantity)

    # 1. Material Cost
    # Volume in cm3 for weight calc
    volume_cm3 = part.volume / 1000.0
    density = material_cfg.get("density_g_cm3", 1.04)
    cost_per_kg = material_cfg.get("cost_per_kg", 20.0)

    mass_kg = (volume_cm3 * density) / 1000.0
    material_cost_per_part = mass_kg * cost_per_kg

    # 2. Machine Time Cost
    # Simple estimation: Time is proportional to volume + overhead
    # In reality, it depends on layer height, infill, etc.
    # For MVP, we assume a deposition rate.
    # Typical FDM: 10-20 mm^3/s. Let's assume 15 mm^3/s = 0.015 cm^3/s.
    deposition_rate_cm3_s = 0.015
    print_time_s = volume_cm3 / deposition_rate_cm3_s
    print_time_hr = print_time_s / 3600.0

    machine_hourly_rate = material_cfg.get("machine_hourly_rate", 20.0)
    machine_cost_per_part = print_time_hr * machine_hourly_rate

    # 3. Setup Cost
    setup_cost = threedp_cfg.costs.get("setup_fee", 10.0)

    # Apply reuse discount
    is_reused = False
    if context is not None:
        part_hash = compute_part_hash(part)
        if part_hash in context:
            setup_cost *= 0.1 # Setup is much cheaper for repeated runs of same part?
            # Actually, for 3D printing, setup is per-print-job usually.
            # But let's keep the pattern of discounting setup for reused parts in a batch context.
            is_reused = True
        context[part_hash] = context.get(part_hash, 0) + quantity

    unit_cost = material_cost_per_part + machine_cost_per_part
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
            "mass_kg": round(mass_kg, 4),
            "print_time_hr": round(print_time_hr, 2),
            "machine_cost_per_unit": round(machine_cost_per_part, 2),
        },
        pricing_explanation=(
            f"3DP cost (${total_cost:.2f}) for {quantity} units. "
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

    # 1. Geometric Validity Check
    if not part.is_valid:
        violations.append(
            "Geometry is not valid (non-manifold or self-intersecting)"
        )

    # 2. Closed Geometry / Watertight Check
    # In build123d/OCP, solids should be closed.
    solids = part.solids()
    if not solids:
        violations.append("Geometry contains no solids")
    else:
        for i, solid in enumerate(solids):
            # Check if solid is closed
            if hasattr(solid, "is_closed") and not solid.is_closed:
                violations.append(f"Solid {i} is not closed (not watertight)")

    # 3. Single Body Check
    if len(solids) > 1:
        violations.append(
            f"Geometry must be a single body, found {len(solids)} solids"
        )

    # 4. Wall Thickness / Thin Features (Optional for now, but could be added)
    # For now, we stick to the basic geometric checks implemented previously.

    # 5. Cost Calculation
    try:
        cost_breakdown = _calculate_3dp_cost(part, config, quantity=1)
    except Exception as e:
        logger.error("3dp_cost_calc_failed", error=str(e))
        # If cost calc fails (e.g. missing config), we should probably fail gracefully or re-raise.
        # But here we return a result with error.
        # Actually, let's let it raise if config is missing, as per other workbenches.
        raise e

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
        """
        Initializes the 3D Print Workbench.

        Args:
            config: Manufacturing configuration. If None, loads default.
        """
        from src.workbenches.config import load_config

        self.config = config or load_config()

    def validate(self, part: Part) -> list[Exception | str]:
        """
        Validates the part for 3D printing.
        """
        result = analyze_3dp(part, self.config)
        return result.violations

    def calculate_cost(
        self,
        part: Part,
        quantity: int = 1,
        context: dict[str, Any] | None = None,
    ) -> CostBreakdown:
        """
        Calculates cost based on part volume, material, and machine time.
        """
        return _calculate_3dp_cost(part, self.config, quantity, context)
