import structlog
from typing import Union
from build123d import Part, Compound

from src.workbenches.models import (
    WorkbenchResult, 
    ManufacturingConfig, 
    ManufacturingMethod
)
from src.workbenches.cnc import analyze_cnc
from src.workbenches.injection_molding import analyze_im

logger = structlog.get_logger()

def validate_and_price(
    part: Union[Part, Compound], 
    method: ManufacturingMethod, 
    config: ManufacturingConfig
) -> WorkbenchResult:
    """
    Unified entry point for DFM (Design for Manufacturing) validation and pricing.
    Dispatches to the appropriate workbench analysis function.
    """
    logger.info("starting_dfm_facade_analysis", method=method)
    
    if method == ManufacturingMethod.CNC:
        return analyze_cnc(part, config)
    elif method == ManufacturingMethod.INJECTION_MOLDING:
        return analyze_im(part, config)
    elif method == ManufacturingMethod.THREE_DP:
        # For 3DP, we use the Workbench class as it doesn't have a functional entry point yet
        from src.workbenches.print_3d import Print3DWorkbench
        wb = Print3DWorkbench()
        violations = wb.validate(part)
        cost = wb.calculate_cost(part)
        
        is_manufacturable = len(violations) == 0
        logger.info("3dp_analysis_complete", is_manufacturable=is_manufacturable)
        
        return WorkbenchResult(
            is_manufacturable=is_manufacturable,
            unit_cost=cost.unit_cost,
            violations=violations,
            metadata={"cost_breakdown": cost.model_dump()}
        )
    else:
        logger.error("unsupported_manufacturing_method", method=method)
        raise ValueError(f"Unsupported manufacturing method: {method}")
