"""
Pydantic schemas for structured data validation across the Problemologist system.

These models define the contracts for:
- objectives.yaml: Central data exchange object between agents
- Review frontmatter: YAML frontmatter for reviewer decisions
"""

from typing import Literal

from pydantic import BaseModel, Field, field_validator, model_validator

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
    """
    A moving part in the environment (motor or passive).
    Used for extraction from assembly.
    """

    part_name: str
    type: Literal["motor", "passive"]
    dofs: list[str]
    control: MotorControl | None = None


class Constraints(BaseModel):
    """Cost and weight constraints for the engineer."""

    max_unit_cost: float
    max_weight: float


class RandomizationMeta(BaseModel):
    """Metadata about randomization for reproducibility."""

    static_variation_id: str | None = None
    runtime_jitter_enabled: bool = True


# =============================================================================
# WP2 Physics Configuration
# =============================================================================


class PhysicsConfig(BaseModel):
    """Configuration for the physics engine."""

    backend: str = "mujoco"  # "mujoco" | "genesis"
    fem_enabled: bool = False
    compute_target: str = "auto"  # "auto" | "cpu" | "gpu"


# =============================================================================
# WP3 Electronics Models
# =============================================================================


class PowerSupplyConfig(BaseModel):
    """Configuration for a DC power supply."""

    type: str = "mains_ac_rectified"
    voltage_dc: float
    max_current_a: float
    location: tuple[float, float, float] | None = None

    @field_validator("location", mode="before")
    @classmethod
    def coerce_list_to_tuple(cls, v):
        if isinstance(v, list):
            return tuple(v)
        return v


class WiringConstraint(BaseModel):
    """Constraints on wire routing and length."""

    max_total_wire_length_mm: float
    restricted_zones: list[ForbidZone] = []


class ElectronicsRequirements(BaseModel):
    """Electronics section for objectives.yaml."""

    power_supply_available: PowerSupplyConfig
    wiring_constraints: WiringConstraint | None = None
    circuit_validation_required: bool = True


class CircuitValidationResult(BaseModel):
    """Result of a circuit validation check."""

    valid: bool
    node_voltages: dict[str, float] = {}
    branch_currents: dict[str, float] = {}
    total_draw_a: float = 0.0
    errors: list[str] = []
    warnings: list[str] = []


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
    physics: PhysicsConfig = PhysicsConfig()
    simulation_bounds: BoundingBox
    moved_object: MovedObject
    constraints: Constraints
    randomization: RandomizationMeta = RandomizationMeta()
    electronics_requirements: ElectronicsRequirements | None = None
    preliminary_totals: dict[str, float] | None = None


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


class AssemblyPartConfig(BaseModel):
    """Configuration for a part in an assembly, including motion metadata."""

    dofs: list[str] = []
    control: MotorControl | None = None


class JointEstimate(BaseModel):
    """Estimate for a joint in the assembly."""

    joint_id: str
    parts: list[str]
    type: str


class SubassemblyEstimate(BaseModel):
    """Estimate for a subassembly within the final assembly."""

    subassembly_id: str
    parts: list[dict[str, AssemblyPartConfig]]
    joints: list[JointEstimate] = []


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


# =============================================================================
# WP3 Assembly Electronics Section
# =============================================================================


class WireTerminal(BaseModel):
    """A terminal on an electronic component."""

    component: str
    terminal: str


class WireConfig(BaseModel):
    """Configuration for a physical wire in the assembly."""

    wire_id: str
    from_terminal: WireTerminal = Field(..., alias="from")
    to_terminal: WireTerminal = Field(..., alias="to")
    gauge_awg: int
    length_mm: float
    routed_in_3d: bool = False

    model_config = {"populate_by_name": True}


class ElectronicComponent(BaseModel):
    """A component in the electronic circuit."""

    component_id: str
    type: Literal["power_supply", "motor", "relay", "connector", "switch"]
    cots_part_id: str | None = None
    assembly_part_ref: str | None = None
    rated_voltage: float | None = None
    stall_current_a: float | None = None


class ElectronicsSection(BaseModel):
    """Electronics section of the assembly definition."""

    power_supply: PowerSupplyConfig
    wiring: list[WireConfig] = []
    components: list[ElectronicComponent] = []


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
    electronics: ElectronicsSection | None = None
    final_assembly: list[SubassemblyEstimate | dict[str, AssemblyPartConfig]] = []
    totals: CostTotals

    @property
    def moving_parts(self) -> list[MovingPart]:
        """Flatten final_assembly to extract all parts with DOFs."""
        parts = []

        def process_item(item):
            if isinstance(item, SubassemblyEstimate):
                for p_dict in item.parts:
                    for name, config in p_dict.items():
                        if config.dofs:
                            parts.append(
                                MovingPart(
                                    part_name=name,
                                    type=("motor" if config.control else "passive"),
                                    dofs=config.dofs,
                                    control=config.control,
                                )
                            )
            elif isinstance(item, dict):
                for name, config in item.items():
                    if config.dofs:
                        parts.append(
                            MovingPart(
                                part_name=name,
                                type=("motor" if config.control else "passive"),
                                dofs=config.dofs,
                                control=config.control,
                            )
                        )

        for item in self.final_assembly:
            process_item(item)
        return parts

    @model_validator(mode="after")
    def validate_caps(self) -> "PreliminaryCostEstimation":
        """Enforce INT-011: Planner target caps must be <= benchmark caps."""
        if (
            self.constraints.planner_target_max_unit_cost_usd
            > self.constraints.benchmark_max_unit_cost_usd
        ):
            raise ValueError(
                f"Planner target cost "
                f"({self.constraints.planner_target_max_unit_cost_usd}) "
                f"must be less than or equal to benchmark max cost "
                f"({self.constraints.benchmark_max_unit_cost_usd})"
            )
        if (
            self.constraints.planner_target_max_weight_kg
            > self.constraints.benchmark_max_weight_kg
        ):
            raise ValueError(
                f"Planner target weight "
                f"({self.constraints.planner_target_max_weight_kg}) "
                f"must be less than or equal to benchmark max weight "
                f"({self.constraints.benchmark_max_weight_kg})"
            )

        # Enforce INT-010: Estimated totals must be within planner target caps
        if (
            self.totals.estimated_unit_cost_usd
            > self.constraints.planner_target_max_unit_cost_usd
        ):
            raise ValueError(
                f"Estimated unit cost "
                f"(${self.totals.estimated_unit_cost_usd}) "
                f"exceeds target limit "
                f"(${self.constraints.planner_target_max_unit_cost_usd})"
            )
        if (
            self.totals.estimated_weight_g / 1000.0
        ) > self.constraints.planner_target_max_weight_kg:
            raise ValueError(
                f"Estimated weight ({self.totals.estimated_weight_g}g) "
                f"exceeds target limit "
                f"({self.constraints.planner_target_max_weight_kg}kg)"
            )

        return self


# Alias for future renaming as per WP3 spec
AssemblyDefinition = PreliminaryCostEstimation
