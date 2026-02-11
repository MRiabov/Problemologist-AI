"""
Pydantic schemas for structured data validation across the Problemologist system.

These models define the contracts for:
- objectives.yaml: Central data exchange object between agents
- Review frontmatter: YAML frontmatter for reviewer decisions
"""

from typing import Literal

from pydantic import BaseModel, field_validator

# =============================================================================
# Common Types
# =============================================================================


class BoundingBox(BaseModel):
    """Axis-aligned bounding box with min/max coordinates."""

    min: tuple[float, float, float]
    max: tuple[float, float, float]

    @field_validator("min", "max", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        """Allow lists to be passed and convert to tuples."""
        if isinstance(v, list):
            return tuple(v)
        return v


# =============================================================================
# objectives.yaml Schema
# =============================================================================


class ForbidZone(BaseModel):
    """A zone that the moved object must not enter."""

    name: str
    min: tuple[float, float, float]
    max: tuple[float, float, float]

    @field_validator("min", "max", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        if isinstance(v, list):
            return tuple(v)
        return v


class ObjectivesSection(BaseModel):
    """The objectives section of objectives.yaml."""

    goal_zone: BoundingBox
    forbid_zones: list[ForbidZone] = []
    build_zone: BoundingBox


class StaticRandomization(BaseModel):
    """Per-benchmark-run randomization of object properties."""

    radius: tuple[float, float] | None = None

    @field_validator("radius", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        if isinstance(v, list):
            return tuple(v)
        return v


class MovedObject(BaseModel):
    """The object that must be guided to the goal zone."""

    label: str
    shape: str
    static_randomization: StaticRandomization = StaticRandomization()
    start_position: tuple[float, float, float]
    runtime_jitter: tuple[float, float, float]

    @field_validator("start_position", "runtime_jitter", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        if isinstance(v, list):
            return tuple(v)
        return v


class MotorControl(BaseModel):
    """Control parameters for motor-type moving parts."""

    mode: Literal["constant", "sinusoidal", "on_off"]
    speed: float
    frequency: float | None = None


class MovingPart(BaseModel):
    """A moving part in the environment (motor or passive)."""

    name: str
    type: Literal["motor", "passive"]
    position: tuple[float, float, float]
    dof: Literal["rotate_x", "rotate_y", "rotate_z", "slide_x", "slide_y", "slide_z"]
    control: MotorControl | None = None
    description: str

    @field_validator("position", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        if isinstance(v, list):
            return tuple(v)
        return v


class Constraints(BaseModel):
    """Cost and weight constraints for the engineer."""

    max_unit_cost: float
    max_weight: float


class RandomizationMeta(BaseModel):
    """Metadata about randomization for reproducibility."""

    static_variation_id: str | None = None
    runtime_jitter_enabled: bool = True


class ObjectivesYaml(BaseModel):
    """
    The objectives.yaml schema - central data exchange object.

    This file defines WHAT the engineer must achieve:
    - Guide the moved_object into the goal_zone
    - Stay WITHIN the build_zone
    - AVOID all forbid_zones
    - Respect max_unit_cost and max_weight constraints
    """

    objectives: ObjectivesSection
    simulation_bounds: BoundingBox
    moved_object: MovedObject
    moving_parts: list[MovingPart] = []
    constraints: Constraints
    randomization: RandomizationMeta = RandomizationMeta()


# =============================================================================
# Review Frontmatter Schema
# =============================================================================


class ReviewFrontmatter(BaseModel):
    """
    YAML frontmatter schema for review documents.

    The decision field controls the agent handover flow:
    - approved: Accept the work, proceed to next stage
    - rejected: Send back for revision with comments
    - confirm_plan_refusal: Confirm CAD agent's refusal of an invalid plan
    - reject_plan_refusal: Override CAD agent's refusal, plan is valid
    """

    decision: Literal[
        "approved", "rejected", "confirm_plan_refusal", "reject_plan_refusal"
    ]
    comments: list[str] = []

    @field_validator("decision")
    @classmethod
    def validate_refusal_decisions(cls, v):
        """Refusal decisions are only valid when CAD agent refused the plan."""
        # Note: Context validation (whether CAD agent actually refused)
        # must be done at the call site, not in the schema
        return v


# =============================================================================
# preliminary_cost_estimation.yaml Schema
# =============================================================================


class ManufacturedPartEstimate(BaseModel):
    """Preliminary estimate for a manufactured part."""

    part_name: str
    part_id: str
    manufacturing_method: str
    material_id: str
    quantity: int
    part_volume_mm3: float
    stock_bbox_mm: dict[str, float]  # {x: float, y: float, z: float}
    stock_volume_mm3: float
    removed_volume_mm3: float
    estimated_unit_cost_usd: float
    pricing_notes: str | None = None


class CotsPartEstimate(BaseModel):
    """Preliminary estimate for a COTS part."""

    part_id: str
    manufacturer: str
    unit_cost_usd: float
    quantity: int
    source: str


class CostTotals(BaseModel):
    """Total estimation for the design."""

    estimated_unit_cost_usd: float
    estimated_weight_g: float
    estimate_confidence: Literal["low", "medium", "high"]


class CostEstimationUnits(BaseModel):
    """Units used in the estimation file."""

    length: str = "mm"
    volume: str = "mm3"
    mass: str = "g"
    currency: str = "USD"


class CostEstimationConstraints(BaseModel):
    """Cap values from benchmark vs planner targets."""

    benchmark_max_unit_cost_usd: float
    benchmark_max_weight_kg: float
    planner_target_max_unit_cost_usd: float
    planner_target_max_weight_kg: float


class PreliminaryCostEstimation(BaseModel):
    """
    Schema for preliminary_cost_estimation.yaml.
    Output by the Engineering Planner to track cost risks.
    """

    version: str = "1.0"
    units: CostEstimationUnits = CostEstimationUnits()
    constraints: CostEstimationConstraints
    manufactured_parts: list[ManufacturedPartEstimate] = []
    cots_parts: list[CotsPartEstimate] = []
    totals: CostTotals
