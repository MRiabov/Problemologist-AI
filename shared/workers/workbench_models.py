from typing import Any

from pydantic import BaseModel, Field

from shared.enums import ManufacturingMethod


class BuildZone(BaseModel):
    """Defines the spatial bounds within which a design must fit."""

    min: tuple[float, float, float]  # (x, y, z) minimum coordinates
    max: tuple[float, float, float]  # (x, y, z) maximum coordinates


class CostBreakdown(BaseModel):
    process: str
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float
    is_reused: bool
    details: dict[str, Any] = Field(default_factory=dict)
    pricing_explanation: str = ""


class WorkbenchMetadata(BaseModel):
    """Standardized metadata for workbench results."""

    cost_breakdown: CostBreakdown | None = None
    dof_count: int | None = None
    dof_warning: str | None = None
    undercut_count: int | None = None
    gate_count: int | None = None
    additional_info: dict[str, Any] = Field(default_factory=dict)


class WorkbenchResult(BaseModel):
    is_manufacturable: bool
    unit_cost: float
    weight_g: float = 0.0
    violations: list[str] = Field(default_factory=list)
    metadata: WorkbenchMetadata = Field(default_factory=WorkbenchMetadata)


class DFMReport(BaseModel):
    overall_status: bool
    results: dict[str, WorkbenchResult]


class MaterialDefinition(BaseModel):
    name: str
    density_g_cm3: float
    density_kg_m3: float | None = None
    cost_per_kg: float
    color: str = "#FFFFFF"
    elongation_stress_mpa: float = 0.0
    restitution: float = 0.5
    friction_coef: float = 0.61  # Default for aluminum
    machine_hourly_rate: float = 0.0
    compatibility: list[ManufacturingMethod] = Field(default_factory=list)

    # NEW - FEM fields (WP2)
    youngs_modulus_pa: float | None = None
    poissons_ratio: float | None = None
    yield_stress_pa: float | None = None
    ultimate_stress_pa: float | None = None
    elongation_at_break: float | None = None
    material_class: str = "rigid"  # "rigid" | "soft" | "elastomer"


class WireDefinition(BaseModel):
    cost_per_m: float


class MethodConfig(BaseModel):
    materials: dict[str, MaterialDefinition]
    constraints: dict[str, Any] = Field(default_factory=dict)
    parameters: dict[str, Any] = Field(default_factory=dict)
    costs: dict[str, Any] = Field(default_factory=dict)


class ManufacturingConfig(BaseModel):
    defaults: dict[str, Any] = Field(default_factory=dict)
    materials: dict[str, MaterialDefinition] = Field(default_factory=dict)
    wires: dict[str, WireDefinition] = Field(default_factory=dict)
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
