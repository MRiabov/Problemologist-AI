"""
Pydantic schemas for structured data validation across the Problemologist system.

These models define the contracts for:
- objectives.yaml: Central data exchange object between agents
- Review frontmatter: YAML frontmatter for reviewer decisions
"""

from typing import Annotated, Any, Literal

from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)

from shared.enums import (
    ElectronicComponentType,
    FailureReason,
    FluidEvalAt,
    FluidObjectiveType,
    FluidShapeType,
    ManufacturingMethod,
    MotorControlMode,
    MovingPartType,
    ReviewDecision,
)
from shared.models.simulation import SimulationFailure
from shared.simulation.schemas import CustomObjectives, SimulatorBackendType

# =============================================================================
# Common Types
# =============================================================================


def _coerce_to_tuple(v: Any) -> Any:
    """Validator to coerce lists to tuples for consistent hashing and immutability."""
    if isinstance(v, list):
        return tuple(v)
    return v


CoercedTuple3D = Annotated[
    tuple[float, float, float], BeforeValidator(_coerce_to_tuple)
]
CoercedTuple2D = Annotated[tuple[float, float], BeforeValidator(_coerce_to_tuple)]


class BoundingBox(BaseModel):
    """Axis-aligned bounding box with min/max coordinates."""

    min: CoercedTuple3D
    max: CoercedTuple3D


# =============================================================================
# WP2 Fluid & Stress Models
# =============================================================================


class FluidProperties(BaseModel):
    viscosity_cp: float = 1.0
    density_kg_m3: float = 1000.0
    surface_tension_n_m: float = 0.07


class FluidVolume(BaseModel):
    type: FluidShapeType
    center: CoercedTuple3D
    # For cylinder
    radius: float | None = None
    height: float | None = None
    # For box
    size: CoercedTuple3D | None = None


class FluidDefinition(BaseModel):
    fluid_id: str
    properties: FluidProperties = FluidProperties()
    initial_volume: FluidVolume
    color: Annotated[tuple[int, int, int], BeforeValidator(_coerce_to_tuple)] = (
        0,
        0,
        200,
    )


class FluidContainmentObjective(BaseModel):
    type: FluidObjectiveType = FluidObjectiveType.FLUID_CONTAINMENT
    fluid_id: str
    containment_zone: BoundingBox
    threshold: float = 0.95
    eval_at: FluidEvalAt = FluidEvalAt.END


class FlowRateObjective(BaseModel):
    type: FluidObjectiveType = FluidObjectiveType.FLOW_RATE
    fluid_id: str
    gate_plane_point: CoercedTuple3D
    gate_plane_normal: CoercedTuple3D
    target_rate_l_per_s: float
    tolerance: float = 0.2


class MaxStressObjective(BaseModel):
    type: Literal["max_stress"] = "max_stress"
    part_label: str
    max_von_mises_mpa: float


# =============================================================================
# objectives.yaml Schema
# =============================================================================


class ForbidZone(BaseModel):
    """A zone that the moved object must not enter."""

    name: str
    min: CoercedTuple3D
    max: CoercedTuple3D


class ObjectivesSection(BaseModel):
    """The objectives section of objectives.yaml."""

    goal_zone: BoundingBox
    forbid_zones: list[ForbidZone] = []
    build_zone: BoundingBox
    fluid_objectives: list[FluidContainmentObjective | FlowRateObjective] = []
    stress_objectives: list[MaxStressObjective] = []


class StaticRandomization(BaseModel):
    """Per-benchmark-run randomization of object properties."""

    radius: CoercedTuple2D | None = None


class MovedObject(BaseModel):
    """The object that must be guided to the goal zone."""

    label: str
    shape: str
    static_randomization: StaticRandomization = StaticRandomization()
    start_position: CoercedTuple3D
    runtime_jitter: CoercedTuple3D


class MotorControl(BaseModel):
    """Control parameters for motor-type moving parts."""

    mode: MotorControlMode
    speed: float
    frequency: float | None = None


class MovingPart(BaseModel):
    """
    A moving part in the environment (motor or passive).
    Used for extraction from assembly.
    """

    part_name: str
    type: MovingPartType
    dofs: list[str]
    control: MotorControl | None = None


class Constraints(BaseModel):
    """Cost and weight constraints for the engineer."""

    max_unit_cost: float
    max_weight_g: float


class RandomizationMeta(BaseModel):
    """Metadata about randomization for reproducibility."""

    static_variation_id: str | None = None
    runtime_jitter_enabled: bool = True


# =============================================================================
# WP2 Physics Configuration
# =============================================================================


class PhysicsConfig(BaseModel):
    """Configuration for the physics engine."""

    backend: SimulatorBackendType = SimulatorBackendType.GENESIS
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
    location: CoercedTuple3D | None = None


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
    failures: list[SimulationFailure] = []
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
    fluids: list[FluidDefinition] = []
    simulation_bounds: BoundingBox
    moved_object: MovedObject
    constraints: Constraints
    randomization: RandomizationMeta = RandomizationMeta()
    electronics_requirements: ElectronicsRequirements | None = None
    assembly_totals: dict[str, float] | None = None


# =============================================================================
# CAD Metadata Models
# =============================================================================


class JointMetadata(BaseModel):
    """Metadata for physical joints attached to parts."""

    type: Literal["hinge", "slide"]
    axis: CoercedTuple3D
    range: CoercedTuple2D | None = None


class PartMetadata(BaseModel):
    """Metadata for individual parts in a CAD assembly."""

    material_id: str | None = None
    cots_id: str | None = None
    is_fixed: bool = Field(default=False, alias="fixed")
    manufacturing_method: ManufacturingMethod | None = None
    joint: JointMetadata | None = None

    model_config = ConfigDict(populate_by_name=True)

    @model_validator(mode="after")
    def validate_identity(self) -> "PartMetadata":
        if not self.material_id and not self.cots_id:
            raise ValueError("PartMetadata must have either material_id or cots_id")
        return self


class CompoundMetadata(BaseModel):
    """Metadata for compounds (assemblies) in a CAD hierarchy."""

    is_fixed: bool = Field(default=False, alias="fixed")
    joint: JointMetadata | None = None

    model_config = ConfigDict(populate_by_name=True)


# =============================================================================
# COTS Catalog Models
# =============================================================================


class COTSMetadata(BaseModel):
    """Metadata for COTS items, indexed into the catalog."""

    part_id: str
    name: str
    category: str
    unit_cost: float
    weight_g: float
    bbox: BoundingBox
    volume: float
    params: dict[str, Any]


# =============================================================================
# Episode & Session Models
# =============================================================================


class EpisodeMetadata(BaseModel):
    """Structured metadata for episodes and benchmark sessions."""

    worker_session_id: str | None = None
    custom_objectives: CustomObjectives | None = None
    detailed_status: str | None = None  # Using str to avoid circular deps if needed
    error: str | None = None
    variant_id: str | None = None
    seed: int | None = None
    prior_episode_id: str | None = None
    is_optimality_check: bool | None = None
    fidelity_check: bool | None = None
    tolerance: float | None = None
    is_reused: bool | None = None
    episode_type: Literal["benchmark", "engineer"] | None = None
    validation_logs: list[str] = Field(default_factory=list)
    prompt: str | None = None
    plan: dict[str, Any] | None = None
    additional_info: dict[str, Any] = Field(default_factory=dict)


# =============================================================================
# Review Frontmatter Schema
# =============================================================================


class ReviewResult(BaseModel):
    """Structured output for the reviewer."""

    decision: ReviewDecision
    reason: str
    required_fixes: list[str] = Field(default_factory=list)


class ReviewFrontmatter(BaseModel):
    """
    YAML frontmatter schema for review documents.

    The decision field controls the agent handover flow:
    - approved: Accept the work, proceed to next stage
    - rejected: Send back for revision with comments
    - confirm_plan_refusal: Confirm CAD agent's refusal of an invalid plan
    - reject_plan_refusal: Override CAD agent's refusal, plan is valid
    """

    decision: ReviewDecision
    comments: list[str] = []

    @field_validator("comments", mode="before")
    @classmethod
    def coerce_comments(cls, v: Any) -> Any:
        if isinstance(v, str):
            return [v]
        return v

    @field_validator("decision")
    @classmethod
    def validate_refusal_decisions(cls, v):
        """Refusal decisions are only valid when CAD agent refused the plan."""
        # Note: Context validation (whether CAD agent actually refused)
        # must be done at the call site, not in the schema
        return v


# =============================================================================
# assembly_definition.yaml Schema
# =============================================================================


class ManufacturedPartEstimate(BaseModel):
    """Assembly estimate for a manufactured part."""

    part_name: str
    part_id: str
    manufacturing_method: ManufacturingMethod
    material_id: str

    @field_validator("manufacturing_method", mode="before")
    @classmethod
    def normalize_method(cls, v: Any) -> Any:
        if isinstance(v, str):
            v_lower = v.lower()
            if v_lower in ["3d_printing", "3dprinting", "3dp"]:
                return ManufacturingMethod.THREE_DP
            if v_lower in ["injection_molding", "im"]:
                return ManufacturingMethod.INJECTION_MOLDING
            # Try to match enum values
            for m in ManufacturingMethod:
                if v_lower == m.value:
                    return m
        return v

    quantity: int
    part_volume_mm3: float
    wall_thickness_mm: float | None = None
    cooling_time_s_estimate: float | None = None
    stock_bbox_mm: dict[str, float]  # {x: float, y: float, z: float}
    stock_volume_mm3: float
    removed_volume_mm3: float
    estimated_unit_cost_usd: float
    pricing_notes: str | None = None
    dfm_suggestions: list[str] = Field(default_factory=list)


class CotsPartEstimate(BaseModel):
    """Assembly estimate for a COTS part."""

    part_id: str
    manufacturer: str
    unit_cost_usd: float
    quantity: int
    source: str


class AssemblyPartConfig(BaseModel):
    """Configuration for a part in an assembly, including motion metadata."""

    dofs: list[str] = []
    control: MotorControl | None = None


class PartConfig(BaseModel):
    """Configuration for a part in an assembly, including motion metadata."""

    name: str
    config: AssemblyPartConfig


class JointEstimate(BaseModel):
    """Estimate for a joint in the assembly."""

    joint_id: str
    parts: list[str]
    type: str


class SubassemblyEstimate(BaseModel):
    """Estimate for a subassembly within the final assembly."""

    subassembly_id: str
    parts: list[PartConfig]
    joints: list[JointEstimate] = []


class CostTotals(BaseModel):
    """Total estimation for the design."""

    estimated_unit_cost_usd: float
    estimated_weight_g: float
    estimate_confidence: Literal["low", "medium", "high"]


class AssemblyUnits(BaseModel):
    """Units used in the estimation file."""

    length: str = "mm"
    volume: str = "mm3"
    mass: str = "g"
    currency: str = "USD"


class AssemblyConstraints(BaseModel):
    """Cap values from benchmark vs planner targets."""

    benchmark_max_unit_cost_usd: float
    benchmark_max_weight_g: float
    planner_target_max_unit_cost_usd: float
    planner_target_max_weight_g: float


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
    waypoints: list[tuple[float, float, float]] = []
    routed_in_3d: bool = False

    model_config = {"populate_by_name": True}

    @field_validator("waypoints", mode="before")
    @classmethod
    def coerce_waypoints(cls, v):
        if isinstance(v, list):
            return [tuple(pt) if isinstance(pt, list) else pt for pt in v]
        return v


class ElectronicComponent(BaseModel):
    """A component in the electronic circuit."""

    component_id: str
    type: ElectronicComponentType
    cots_part_id: str | None = None
    assembly_part_ref: str | None = None
    rated_voltage: float | None = None
    stall_current_a: float | None = None


class ElectronicsSection(BaseModel):
    """Electronics section of the assembly definition."""

    power_supply: PowerSupplyConfig
    wiring: list[WireConfig] = []
    components: list[ElectronicComponent] = []


class AssemblyDefinition(BaseModel):
    """
    Schema for assembly_definition.yaml.
    Output by the Engineering Planner to track cost risks.
    """

    version: str = "1.0"
    units: AssemblyUnits = AssemblyUnits()
    constraints: AssemblyConstraints
    manufactured_parts: list[ManufacturedPartEstimate] = []
    cots_parts: list[CotsPartEstimate] = []
    electronics: ElectronicsSection | None = None
    final_assembly: list[SubassemblyEstimate | PartConfig] = []
    totals: CostTotals
    dfm_suggestions: list[str] = Field(default_factory=list)

    @property
    def moving_parts(self) -> list[MovingPart]:
        """Flatten final_assembly to extract all parts with DOFs."""
        parts = []

        def process_item(item):
            if isinstance(item, SubassemblyEstimate):
                for p_config in item.parts:
                    if p_config.config.dofs:
                        parts.append(
                            MovingPart(
                                part_name=p_config.name,
                                type=(
                                    "motor" if p_config.config.control else "passive"
                                ),
                                dofs=p_config.config.dofs,
                                control=p_config.config.control,
                            )
                        )
            elif isinstance(item, PartConfig) and item.config.dofs:
                parts.append(
                    MovingPart(
                        part_name=item.name,
                        type=("motor" if item.config.control else "passive"),
                        dofs=item.config.dofs,
                        control=item.config.control,
                    )
                )

        for item in self.final_assembly:
            process_item(item)
        return parts

    @model_validator(mode="after")
    def validate_caps(self) -> "AssemblyDefinition":
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
            self.constraints.planner_target_max_weight_g
            > self.constraints.benchmark_max_weight_g
        ):
            raise ValueError(
                f"Planner target weight "
                f"({self.constraints.planner_target_max_weight_g}g) "
                f"must be less than or equal to benchmark max weight "
                f"({self.constraints.benchmark_max_weight_g}g)"
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
            self.totals.estimated_weight_g
        ) > self.constraints.planner_target_max_weight_g:
            raise ValueError(
                f"Estimated weight ({self.totals.estimated_weight_g}g) "
                f"exceeds target limit "
                f"({self.constraints.planner_target_max_weight_g}g)"
            )

        return self
