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


class BuildZone(BaseModel):
    """Defines the spatial bounds within which a design must fit."""

    min: tuple[float, float, float]  # (x, y, z) minimum coordinates
    max: tuple[float, float, float]  # (x, y, z) maximum coordinates


class DFMReport(BaseModel):
    overall_status: StrictBool
    results: dict[StrictStr, WorkbenchResult]


class MaterialDefinition(BaseModel):
    name: StrictStr
    density_g_cm3: StrictFloat
    density_kg_m3: StrictFloat | None = None
    cost_per_kg: StrictFloat
    color: StrictStr = "#FFFFFF"
    elongation_stress_mpa: StrictFloat = 0.0
    restitution: StrictFloat = 0.5
    friction_coef: StrictFloat = 0.5
    machine_hourly_rate: StrictFloat = 0.0
    compatibility: list[ManufacturingMethod] = Field(default_factory=list)

    # NEW - FEM fields (WP2)
    youngs_modulus_pa: StrictFloat | None = None
    poissons_ratio: StrictFloat | None = None
    yield_stress_pa: StrictFloat | None = None
    ultimate_stress_pa: StrictFloat | None = None
    elongation_at_break: StrictFloat | None = None
    material_class: StrictStr = "rigid" # "rigid" | "soft" | "elastomer"


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
    materials: dict[StrictStr, MaterialDefinition]
    constraints: dict[StrictStr, Any] = Field(default_factory=dict)
    parameters: dict[StrictStr, Any] = Field(default_factory=dict)
    costs: dict[StrictStr, Any] = Field(default_factory=dict)


class ManufacturingConfig(BaseModel):
    defaults: dict[StrictStr, Any] = Field(default_factory=dict)
    materials: dict[StrictStr, MaterialDefinition] = Field(default_factory=dict)
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
