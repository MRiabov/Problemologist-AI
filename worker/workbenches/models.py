from enum import Enum
from typing import Any

from pydantic import BaseModel, Field, StrictBool, StrictFloat, StrictStr


class ManufacturingMethod(str, Enum):
    CNC = "cnc"
    THREE_DP = "3dp"
    INJECTION_MOLDING = "im"


class WorkbenchResult(BaseModel):
    is_manufacturable: StrictBool
    unit_cost: StrictFloat
    violations: list[StrictStr] = Field(default_factory=list)
    metadata: dict[StrictStr, Any] = Field(default_factory=dict)


class DFMReport(BaseModel):
    overall_status: StrictBool
    results: dict[StrictStr, WorkbenchResult]


class MaterialDefinition(BaseModel):
    name: StrictStr
    density_g_cm3: StrictFloat
    cost_per_kg: StrictFloat
    compatibility: list[ManufacturingMethod] = Field(default_factory=list)


class CostBreakdown(BaseModel):
    process: StrictStr
    total_cost: StrictFloat
    unit_cost: StrictFloat
    material_cost_per_unit: StrictFloat
    setup_cost: StrictFloat
    is_reused: StrictBool
    details: dict[StrictStr, Any] = Field(default_factory=dict)
    pricing_explanation: StrictStr = ""


class MethodConfig(BaseModel):
    materials: dict[StrictStr, dict[StrictStr, Any]]
    constraints: dict[StrictStr, Any] = Field(default_factory=dict)
    parameters: dict[StrictStr, Any] = Field(default_factory=dict)
    costs: dict[StrictStr, Any] = Field(default_factory=dict)


class ManufacturingConfig(BaseModel):
    defaults: dict[StrictStr, Any] = Field(default_factory=dict)
    cnc: MethodConfig | None = None
    injection_molding: MethodConfig | None = None
    three_dp: MethodConfig | None = None

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
