from enum import Enum
from typing import Any
from pydantic import BaseModel, ConfigDict, Field


class CostBreakdown(BaseModel):
    model_config = ConfigDict(extra="ignore")

    process: str  # enum?
    total_cost: float
    unit_cost: float
    material_cost_per_unit: float
    setup_cost: float = 0.0
    is_reused: bool = False
    details: dict[str, Any] = Field(default_factory=dict)
    pricing_explanation: str = ""


class ValidationSeverity(str, Enum):
    ERROR = "error"
    WARNING = "warning"


class ValidationViolation(BaseModel):
    description: str
    severity: ValidationSeverity = ValidationSeverity.ERROR


class ValidationStatus(str, Enum):
    PASSED = "pass"
    FAILED = "fail"


class ValidationReport(BaseModel):
    model_config = ConfigDict(extra="ignore")

    status: ValidationStatus
    manufacturability_score: float
    violations: list[ValidationViolation]
    cost_analysis: CostBreakdown
    parts: list[dict[str, Any]] = Field(default_factory=list)
    stl_path: str | None = None
    error: str | None = None
