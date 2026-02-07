from enum import Enum
from typing import Any, Dict, List, Optional, Union
from pydantic import BaseModel, Field, StrictStr, StrictInt, StrictFloat, StrictBool

class ManufacturingMethod(str, Enum):
    CNC = "cnc"
    THREE_DP = "3dp"
    INJECTION_MOLDING = "im"

class WorkbenchResult(BaseModel):
    is_manufacturable: StrictBool
    unit_cost: StrictFloat
    violations: List[StrictStr] = Field(default_factory=list)
    metadata: Dict[StrictStr, Any] = Field(default_factory=dict)

class DFMReport(BaseModel):
    overall_status: StrictBool
    results: Dict[StrictStr, WorkbenchResult]

class MaterialDefinition(BaseModel):
    name: StrictStr
    density_g_cm3: StrictFloat
    cost_per_kg: StrictFloat
    compatibility: List[ManufacturingMethod] = Field(default_factory=list)

class CostBreakdown(BaseModel):
    process: StrictStr
    total_cost: StrictFloat
    unit_cost: StrictFloat
    material_cost_per_unit: StrictFloat
    setup_cost: StrictFloat
    is_reused: StrictBool
    details: Dict[StrictStr, Any] = Field(default_factory=dict)
    pricing_explanation: StrictStr = ""

class MethodConfig(BaseModel):
    materials: Dict[StrictStr, Dict[StrictStr, Any]]
    constraints: Dict[StrictStr, Any] = Field(default_factory=dict)
    parameters: Dict[StrictStr, Any] = Field(default_factory=dict)
    costs: Dict[StrictStr, Any] = Field(default_factory=dict)

class ManufacturingConfig(BaseModel):
    defaults: Dict[StrictStr, Any] = Field(default_factory=dict)
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