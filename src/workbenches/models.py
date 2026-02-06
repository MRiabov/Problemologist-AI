from dataclasses import dataclass, field
from typing import Dict, Any

@dataclass
class CostBreakdown:
    process: str
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float
    is_reused: bool
    details: Dict[str, Any] = field(default_factory=dict)
    pricing_explanation: str = ""
