from enum import Enum
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field

class ManufacturingMethod(str, Enum):
    CNC = "cnc"
    THREE_DP = "3dp"
    INJECTION_MOLDING = "im"

class WorkbenchResult(BaseModel):
    is_manufacturable: bool
    unit_cost: float
    violations: List[str] = Field(default_factory=list)
    metadata: Dict[str, Any] = Field(default_factory=dict)

class DFMReport(BaseModel):
    overall_status: bool
    results: Dict[str, WorkbenchResult]

class MaterialDefinition(BaseModel):
    name: str
    density_g_cm3: float
    cost_per_kg: float
    compatibility: List[ManufacturingMethod] = Field(default_factory=list)

class CostBreakdown(BaseModel):
    process: str
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float
    is_reused: bool
    details: Dict[str, Any] = Field(default_factory=dict)
    pricing_explanation: str = ""

class MethodConfig(BaseModel):
    materials: Dict[str, Dict[str, Any]]
    constraints: Dict[str, Any] = Field(default_factory=dict)
    parameters: Dict[str, Any] = Field(default_factory=dict)
    costs: Dict[str, Any] = Field(default_factory=dict)

class ManufacturingConfig(BaseModel):
    defaults: Dict[str, Any] = Field(default_factory=dict)
    cnc: Optional[MethodConfig] = None
    injection_molding: Optional[MethodConfig] = None
    three_dp: Optional[MethodConfig] = None

    def __getitem__(self, key: str) -> Any:
        # Support dict-like access for backward compatibility with existing workbenches
        if key == "cnc":
            return self.cnc.model_dump() if self.cnc else {}
        if key == "injection_molding":
            return self.injection_molding.model_dump() if self.injection_molding else {}
        if key == "three_dp":
            return self.three_dp.model_dump() if self.three_dp else {}
        if key == "defaults":
            return self.defaults
        raise KeyError(key)

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self[key]
        except KeyError:
            return default