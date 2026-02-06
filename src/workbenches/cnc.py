import structlog
from typing import Union, List, Dict, Any, Optional
from build123d import Part, Compound

from src.workbenches.models import (
    WorkbenchResult, 
    ManufacturingConfig, 
    CostBreakdown,
    ManufacturingMethod
)
from src.workbenches.analysis_utils import (
    check_undercuts,
    compute_part_hash,
    part_to_trimesh,
)
from src.workbenches.base import Workbench

logger = structlog.get_logger()

def check_internal_corner_radii(part: Union[Part, Compound], min_radius: float = 1.0) -> List[str]:
    """
    Checks for internal corners that have a radius smaller than the minimum tool radius.
    In 3-axis CNC, sharp internal vertical corners are impossible to machine.
    
    Note: This is a simplified implementation for the prototype.
    """
    violations = []
    # Placeholder: In a real implementation, we would iterate over edges,
    # identify concave edges, and check if they are filleted with radius >= min_radius.
    # For now, we'll log that this check is active.
    logger.debug("checking_internal_corner_radii", min_radius=min_radius)
    
    # Logic to be implemented:
    # 1. Identify all edges parallel to the Z axis (vertical edges).
    # 2. For each vertical edge, check if it is "concave" (internal corner).
    # 3. If concave, check if it's a circular arc and its radius.
    
    return violations

def calculate_cnc_cost(
    part: Union[Part, Compound], 
    config: ManufacturingConfig,
    quantity: int = 1, 
    context: Optional[Dict[str, Any]] = None
) -> CostBreakdown:
    """
    Calculates CNC cost: Setup + (Material + Run) * Quantity.
    """
    cnc_cfg = config.cnc
    if not cnc_cfg:
        raise ValueError("CNC configuration missing")
        
    # Default to aluminum_6061 for now
    material_name = config.defaults.get("material", "aluminum_6061")
    if material_name not in cnc_cfg.materials:
        material_name = list(cnc_cfg.materials.keys())[0]
        
    material_cfg = cnc_cfg.materials[material_name]
    mrr = cnc_cfg.constraints.get("mrr_mm3_per_min", 1000.0)
    
    logger.info("calculating_cnc_cost", material=material_name, quantity=quantity)

    # 0. Stock / Material Analysis
    bbox = part.bounding_box()
    stock_dims = (bbox.size.X, bbox.size.Y, bbox.size.Z)
    stock_volume_cm3 = (stock_dims[0] * stock_dims[1] * stock_dims[2]) / 1000.0
    part_volume_cm3 = part.volume / 1000.0
    removed_volume_cm3 = max(0, stock_volume_cm3 - part_volume_cm3)

    # 1. Material Cost
    density = material_cfg.get("density_g_cm3", 2.7)
    cost_per_kg = material_cfg.get("cost_per_kg", 6.0)
    stock_mass_kg = (stock_volume_cm3 * density) / 1000.0
    material_cost_per_part = stock_mass_kg * cost_per_kg

    # 2. Run Cost
    # Machining time driven by removed volume + finishing
    roughing_time_min = (removed_volume_cm3 * 1000.0) / mrr
    
    params = cnc_cfg.parameters
    finishing_feed_rate = params.get("finishing_feed_rate_mm_min", 500.0)
    finishing_stepover = params.get("finishing_stepover_mm", 0.5)
    
    surface_area_mm2 = part.area
    finishing_time_min = surface_area_mm2 / (finishing_feed_rate * finishing_stepover)
    
    machining_time_min = roughing_time_min + finishing_time_min
    hourly_rate = material_cfg.get("machine_hourly_rate", 80.0)
    run_cost_per_part = (machining_time_min / 60.0) * hourly_rate

    # 3. Setup Cost
    setup_cost = cnc_cfg.costs.get("setup_fee", hourly_rate)

    # Apply reuse discount if part hash is in context
    is_reused = False
    if context is not None:
        part_hash = compute_part_hash(part)
        if part_hash in context:
            setup_cost *= 0.5
            is_reused = True
        context[part_hash] = context.get(part_hash, 0) + quantity

    total_cost = setup_cost + (material_cost_per_part + run_cost_per_part) * quantity

    return CostBreakdown(
        process="cnc_milling",
        total_cost=total_cost,
        unit_cost=total_cost / quantity if quantity > 0 else 0.0,
        material_cost_per_unit=round(material_cost_per_part, 2),
        setup_cost=round(setup_cost, 2),
        is_reused=is_reused,
        details={
            "stock_dims_mm": [round(d, 2) for d in stock_dims],
            "stock_volume_cm3": round(stock_volume_cm3, 2),
            "part_volume_cm3": round(part_volume_cm3, 2),
            "removed_volume_cm3": round(removed_volume_cm3, 2),
            "machining_time_min": round(machining_time_min, 2),
            "run_cost_per_unit": round(run_cost_per_part, 2),
        },
        pricing_explanation=(
            f"CNC cost (${total_cost:.2f}) for {quantity} units. "
            f"Material: {material_name} (${material_cost_per_part:.2f}/unit). "
            f"Machining time: {machining_time_min:.2f} min. "
            f"Setup fee: ${setup_cost:.2f}."
        )
    )

def analyze_cnc(part: Union[Part, Compound], config: ManufacturingConfig) -> WorkbenchResult:
    """
    Functional entry point for CNC analysis.
    """
    logger.info("starting_cnc_analysis")
    
    violations = []
    
    # 1. Undercut Check
    mesh = part_to_trimesh(part)
    undercuts = check_undercuts(mesh, (0, 0, 1))
    if undercuts:
        msg = f"CNC Machining Violation: {len(undercuts)} undercut faces detected."
        logger.warning("cnc_undercuts_detected", count=len(undercuts))
        violations.append(msg)
        
    # 2. Internal Corner Check
    min_radius = config.cnc.constraints.get("min_tool_radius_mm", 1.0) if config.cnc else 1.0
    corner_violations = check_internal_corner_radii(part, min_radius)
    violations.extend(corner_violations)
    
    # 3. Cost Calculation (single unit for analysis)
    cost_breakdown = calculate_cnc_cost(part, config, quantity=1)
    
    is_manufacturable = len(violations) == 0
    
    logger.info("cnc_analysis_complete", is_manufacturable=is_manufacturable, violations=len(violations))
    
    return WorkbenchResult(
        is_manufacturable=is_manufacturable,
        unit_cost=cost_breakdown.unit_cost,
        violations=violations,
        metadata={
            "cost_breakdown": cost_breakdown.model_dump(),
            "undercut_count": len(undercuts)
        }
    )

class CNCWorkbench(Workbench):
    """
    CNC Milling Workbench for 3-axis machining (Workbench Class wrapper).
    """

    def __init__(self, config: Optional[ManufacturingConfig] = None):
        from src.workbenches.config import load_config
        self.config = config or load_config()

    def validate(self, part: Part) -> List[Union[Exception, str]]:
        result = analyze_cnc(part, self.config)
        return result.violations

    def calculate_cost(
        self, part: Part, quantity: int = 1, context: Optional[Dict[str, Any]] = None
    ) -> CostBreakdown:
        return calculate_cnc_cost(part, self.config, quantity, context)