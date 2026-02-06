from dataclasses import dataclass
from typing import Any

@dataclass
class CostBreakdown:
    process: str
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float
    is_reused: bool
    details: dict[str, Any]
    pricing_explanation: str
