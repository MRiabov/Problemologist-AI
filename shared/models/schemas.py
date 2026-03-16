"""
Pydantic schemas for structured data validation across the Problemologist system.

These models define the contracts for:
- benchmark_definition.yaml: Central benchmark exchange object between agents
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
    AgentName,
    BenchmarkAttachmentMethod,
    BenchmarkRefusalReason,
    ElectricalRefusalReason,
    ElectronicComponentType,
    EntryFailureDisposition,
    EpisodePhase,
    EpisodeType,
    FailureClass,
    FluidEvalAt,
    FluidObjectiveType,
    FluidShapeType,
    GenerationKind,
    ManufacturingMethod,
    MechanicalRefusalReason,
    MotorControlMode,
    MovingPartType,
    ReviewDecision,
    SeedMatchMethod,
    TerminalReason,
)
from shared.models.simulation import SimulationFailure
from shared.simulation.schemas import (
    CustomObjectives,
    SimulatorBackendType,
    get_default_simulator_backend,
)

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


class StrictContractModel(BaseModel):
    """Strict contract model that rejects unknown fields."""

    model_config = ConfigDict(extra="forbid")


class CodeReference(BaseModel):
    """Reference to a specific line range in a file."""

    file_path: str
    start_line: int
    end_line: int


class BackupParams(BaseModel):
    """Parameters for backup workflow."""

    db_url: str | None = None
    s3_bucket: str | None = None
    source_bucket: str | None = None
    backup_bucket: str | None = None


class BackupResult(BaseModel):
    """Result of a backup operation."""

    postgres_backup_key: str | None = None
    s3_files_backed_up: int | None = None


class SchematicItem(BaseModel):
    """An item in the electronics schematic representation."""

    type: str

    model_config = ConfigDict(extra="allow")


class BoundingBox(StrictContractModel):
    """Axis-aligned bounding box with min/max coordinates."""

    min: CoercedTuple3D
    max: CoercedTuple3D


# =============================================================================
# WP2 Fluid & Stress Models
# =============================================================================


class FluidProperties(StrictContractModel):
    viscosity_cp: float = 1.0
    density_kg_m3: float = 1000.0
    surface_tension_n_m: float = 0.07


class FluidVolume(StrictContractModel):
    type: FluidShapeType
    center: CoercedTuple3D
    # For cylinder
    radius: float | None = None
    height: float | None = None
    # For box
    size: CoercedTuple3D | None = None


class FluidDefinition(StrictContractModel):
    fluid_id: str
    properties: FluidProperties = FluidProperties()
    initial_volume: FluidVolume
    color: Annotated[tuple[int, int, int], BeforeValidator(_coerce_to_tuple)] = (
        0,
        0,
        200,
    )


class FluidContainmentObjective(StrictContractModel):
    type: FluidObjectiveType = FluidObjectiveType.FLUID_CONTAINMENT
    fluid_id: str
    containment_zone: BoundingBox
    threshold: float = 0.95
    eval_at: FluidEvalAt = FluidEvalAt.END


class FlowRateObjective(StrictContractModel):
    type: FluidObjectiveType = FluidObjectiveType.FLOW_RATE
    fluid_id: str
    gate_plane_point: CoercedTuple3D
    gate_plane_normal: CoercedTuple3D
    target_rate_l_per_s: float
    tolerance: float = 0.2


class MaxStressObjective(StrictContractModel):
    type: Literal["max_stress"] = "max_stress"
    part_label: str
    max_von_mises_mpa: float


# =============================================================================
# benchmark_definition.yaml Schema
# =============================================================================


class ForbidZone(StrictContractModel):
    """A zone that the moved object must not enter."""

    name: str
    min: CoercedTuple3D
    max: CoercedTuple3D


class ObjectivesSection(StrictContractModel):
    """The objective section of benchmark_definition.yaml."""

    goal_zone: BoundingBox
    forbid_zones: list[ForbidZone] = []
    build_zone: BoundingBox
    fluid_objectives: list[FluidContainmentObjective | FlowRateObjective] = []
    stress_objectives: list[MaxStressObjective] = []


class StaticRandomization(StrictContractModel):
    """Per-benchmark-run randomization of object properties."""

    radius: CoercedTuple2D | None = None


class MovedObject(StrictContractModel):
    """The object that must be guided to the goal zone."""

    label: str
    shape: str
    static_randomization: StaticRandomization = StaticRandomization()
    start_position: CoercedTuple3D
    runtime_jitter: CoercedTuple3D


class MotorControl(StrictContractModel):
    """Control parameters for motor-type moving parts."""

    mode: MotorControlMode
    speed: float
    frequency: float | None = None


class MovingPart(StrictContractModel):
    """
    A moving part in the environment (motor or passive).
    Used for extraction from assembly.
    """

    part_name: str
    type: MovingPartType
    dofs: list[str]
    control: MotorControl | None = None


class Constraints(StrictContractModel):
    """Cost and weight constraints for the engineer."""

    max_unit_cost: float
    max_weight_g: float
    target_quantity: int | None = None


class RandomizationMeta(StrictContractModel):
    """Metadata about randomization for reproducibility."""

    static_variation_id: str | None = None
    runtime_jitter_enabled: bool = True


# =============================================================================
# Simulation Scene Definition Models (Input)
# =============================================================================


class EntityDefinition(BaseModel):
    name: str
    type: str = "rigid"  # "rigid", "soft_mesh", "zone", etc.
    file: str | None = None
    pos: CoercedTuple3D = (0.0, 0.0, 0.0)
    euler: CoercedTuple3D = (0.0, 0.0, 0.0)
    material_id: str = "aluminum_6061"
    is_zone: bool = False

    # For zones
    min: CoercedTuple3D | None = None
    max: CoercedTuple3D | None = None
    size: CoercedTuple3D | None = None

    model_config = ConfigDict(extra="allow")


class MotorDefinition(BaseModel):
    name: str | None = None
    part_name: str | None = None
    joint: str | None = None
    type: str = "servo"
    parameters: dict[str, Any] = Field(default_factory=dict)

    model_config = ConfigDict(extra="allow")


class CableDefinition(BaseModel):
    wire_id: str
    points: list[CoercedTuple3D]
    radius: float = 0.001
    material_id: str = "copper"

    model_config = ConfigDict(extra="allow")


class SceneDefinition(BaseModel):
    entities: list[EntityDefinition] = Field(default_factory=list)
    motors: list[MotorDefinition] = Field(default_factory=list)
    cables: list[CableDefinition] = Field(default_factory=list)
    fluids: list[FluidDefinition] = Field(default_factory=list)

    model_config = ConfigDict(extra="allow")


# =============================================================================
# WP2 Physics Configuration
# =============================================================================


class PhysicsConfig(StrictContractModel):
    """Configuration for the physics engine."""

    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)
    fem_enabled: bool = False
    compute_target: str = "auto"  # "auto" | "cpu" | "gpu"


# =============================================================================
# WP3 Electronics Models
# =============================================================================


class PowerSupplyConfig(StrictContractModel):
    """Configuration for a DC power supply."""

    type: str = "mains_ac_rectified"
    voltage_dc: float
    max_current_a: float
    location: CoercedTuple3D | None = None


class WiringConstraint(StrictContractModel):
    """Constraints on wire routing and length."""

    max_total_wire_length_mm: float
    restricted_zones: list[ForbidZone] = []


class ElectronicsRequirements(StrictContractModel):
    """Electronics requirements section for benchmark_definition.yaml."""

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


class BenchmarkPartDrillPolicy(StrictContractModel):
    """Benchmark-owned drilling policy for an environment part."""

    allowed: bool = False
    max_hole_count: int | None = None
    diameter_range_mm: CoercedTuple2D | None = None
    max_depth_mm: float | None = None
    notes: str | None = None

    @model_validator(mode="after")
    def validate_contract(self) -> "BenchmarkPartDrillPolicy":
        if self.max_hole_count is not None and self.max_hole_count < 1:
            raise ValueError("max_hole_count must be >= 1")
        if self.diameter_range_mm is not None:
            diameter_min, diameter_max = self.diameter_range_mm
            if diameter_min <= 0 or diameter_max <= 0:
                raise ValueError("diameter_range_mm values must be > 0")
            if diameter_min > diameter_max:
                raise ValueError("diameter_range_mm minimum must be <= maximum")
        if self.max_depth_mm is not None and self.max_depth_mm <= 0:
            raise ValueError("max_depth_mm must be > 0")
        if not self.allowed and (
            self.max_hole_count is not None
            or self.diameter_range_mm is not None
            or self.max_depth_mm is not None
        ):
            raise ValueError("drill policy constraints require allowed=true")
        return self


class BenchmarkPartAttachmentPolicy(StrictContractModel):
    """Benchmark-owned attachment policy for an environment part."""

    attachment_methods: list[BenchmarkAttachmentMethod] = Field(default_factory=list)
    drill_policy: BenchmarkPartDrillPolicy | None = None
    notes: str | None = None

    @field_validator("attachment_methods")
    @classmethod
    def validate_attachment_methods(
        cls, value: list[BenchmarkAttachmentMethod]
    ) -> list[BenchmarkAttachmentMethod]:
        if not value:
            raise ValueError(
                "attachment_methods must contain at least one method when attachment_policy is declared"
            )
        if len(value) != len(set(value)):
            raise ValueError("attachment_methods must not contain duplicates")
        if BenchmarkAttachmentMethod.NONE in value and len(value) != 1:
            raise ValueError(
                "attachment_methods 'none' must be the only method when present"
            )
        return value

    @model_validator(mode="after")
    def validate_drill_contract(self) -> "BenchmarkPartAttachmentPolicy":
        if (
            self.drill_policy is not None
            and self.drill_policy.allowed
            and BenchmarkAttachmentMethod.FASTENER not in self.attachment_methods
        ):
            raise ValueError(
                "drill_policy.allowed=true requires attachment_methods to include 'fastener'"
            )
        return self


class BenchmarkPartMetadata(StrictContractModel):
    """Benchmark-owned metadata for environment and fixture parts."""

    fixed: bool = False
    material_id: str | None = None
    cots_id: str | None = None
    attachment_policy: BenchmarkPartAttachmentPolicy | None = None

    @model_validator(mode="after")
    def validate_identity(self) -> "BenchmarkPartMetadata":
        if not self.material_id and not self.cots_id:
            raise ValueError(
                "BenchmarkPartMetadata must have either material_id or cots_id"
            )
        return self


class BenchmarkPartDefinition(StrictContractModel):
    """Metadata declaration for a benchmark-owned part or fixture."""

    part_id: str
    label: str
    metadata: BenchmarkPartMetadata


class BenchmarkDefinition(StrictContractModel):
    """
    The benchmark_definition.yaml schema - central benchmark exchange object.

    This file defines WHAT the engineer must achieve and the benchmark-owned
    environment metadata:
    - Guide the moved_object into the goal_zone
    - Stay WITHIN the build_zone
    - AVOID all forbid_zones
    - Respect max_unit_cost and max_weight constraints
    - Reference benchmark-owned part metadata for fixtures and environment parts
    """

    objectives: ObjectivesSection
    benchmark_parts: list[BenchmarkPartDefinition] = Field(default_factory=list)
    physics: PhysicsConfig = PhysicsConfig()
    fluids: list[FluidDefinition] = []
    simulation_bounds: BoundingBox
    moved_object: MovedObject
    constraints: Constraints
    randomization: RandomizationMeta = RandomizationMeta()
    electronics_requirements: ElectronicsRequirements | None = None
    assembly_totals: dict[str, float] | None = None

    @model_validator(mode="after")
    def validate_part_identity_uniqueness(self) -> "BenchmarkDefinition":
        part_ids = [part.part_id for part in self.benchmark_parts]
        labels = [part.label for part in self.benchmark_parts]
        if len(part_ids) != len(set(part_ids)):
            raise ValueError("benchmark_parts.part_id values must be unique")
        if len(labels) != len(set(labels)):
            raise ValueError("benchmark_parts.label values must be unique")
        return self


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
    benchmark_id: str | None = None
    custom_objectives: CustomObjectives | None = None
    detailed_status: str | None = None  # Using str to avoid circular deps if needed
    episode_phase: EpisodePhase | None = None
    terminal_reason: TerminalReason | None = None
    failure_class: FailureClass | None = None
    error: str | None = None
    variant_id: str | None = None
    seed: int | None = None
    prior_episode_id: str | None = None
    is_optimality_check: bool | None = None
    fidelity_check: bool | None = None
    tolerance: float | None = None
    is_reused: bool | None = None
    episode_type: EpisodeType | None = None
    validation_logs: list[str] = Field(default_factory=list)
    prompt: str | None = None
    plan: dict[str, Any] | None = None
    cost: float | None = None
    weight: float | None = None
    seed_id: str | None = None
    seed_dataset: str | None = None
    seed_match_method: SeedMatchMethod | None = None
    generation_kind: GenerationKind | None = None
    parent_seed_id: str | None = None
    is_integration_test: bool | None = None
    integration_test_id: str | None = None
    disable_sidecars: bool | None = None
    additional_info: dict[str, Any] = Field(default_factory=dict)


class TraceMetadata(BaseModel):
    """Structured metadata for individual traces (tool calls, nodes, etc.)."""

    tool_name: str | None = None
    tool_args: dict[str, Any] | None = None
    observation: str | None = None
    prediction: str | None = None
    error: str | None = None

    # Simulation specific
    simulation_run_id: str | None = None
    backend: SimulatorBackendType | None = None

    # COTS specific
    cots_query_id: str | None = None

    # Review specific
    review_id: str | None = None
    decision: ReviewDecision | None = None
    checklist: dict[str, str | float | bool] = Field(default_factory=dict)

    # Reasoning trace specific
    reasoning_step_index: int | None = None
    reasoning_source: str | None = None

    additional_info: dict[str, Any] = Field(default_factory=dict)


class PlannerSubmissionResult(BaseModel):
    """Typed result contract for explicit planner handoff submission."""

    ok: bool
    status: Literal["submitted", "rejected"]
    errors: list[str] = Field(default_factory=list)
    node_type: AgentName


class EntryValidationError(BaseModel):
    """Structured entry-validation error payload persisted in episode metadata."""

    code: str
    message: str
    source: str
    artifact_path: str | None = None


class EntryValidationContext(BaseModel):
    """Structured node-entry validation context persisted in episode metadata."""

    node: AgentName | None = None
    disposition: EntryFailureDisposition | None = None
    reason_code: str | None = None
    reroute_target: AgentName | None = None
    errors: list[EntryValidationError] = Field(default_factory=list)


# =============================================================================
# Review Frontmatter Schema
# =============================================================================


class ReviewResult(StrictContractModel):
    """Structured output for the reviewer."""

    decision: ReviewDecision
    reason: str
    required_fixes: list[str] = Field(default_factory=list)
    checklist: dict[str, str | float | bool] = Field(default_factory=dict)

    @field_validator("decision", mode="before")
    @classmethod
    def normalize_decision(cls, v: Any) -> Any:
        if isinstance(v, str):
            return v.upper()
        return v


class ReviewFrontmatter(StrictContractModel):
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
    evidence: dict[str, Any] | None = None

    @field_validator("decision", mode="before")
    @classmethod
    def normalize_decision(cls, v: Any) -> Any:
        if isinstance(v, str):
            return v.upper()
        return v

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


class PlanRefusalFrontmatter(StrictContractModel):
    """
    YAML frontmatter for plan_refusal.md.
    Refusal logic must be structured, role-specific, and machine-validated.
    """

    reasons: list[
        MechanicalRefusalReason | ElectricalRefusalReason | BenchmarkRefusalReason
    ]
    role: AgentName

    @model_validator(mode="after")
    def validate_role_reasons(self) -> "PlanRefusalFrontmatter":
        """Validate that reasons match the role."""
        if self.role == AgentName.ENGINEER_CODER:
            if not all(
                isinstance(r, MechanicalRefusalReason)
                or isinstance(r, ElectricalRefusalReason)
                or str(r) in MechanicalRefusalReason.__members__
                or str(r) in ElectricalRefusalReason.__members__
                for r in self.reasons
            ):
                raise ValueError(f"Invalid reasons for {self.role}")
        elif self.role == AgentName.BENCHMARK_CODER and not all(
            isinstance(r, BenchmarkRefusalReason)
            or str(r) in BenchmarkRefusalReason.__members__
            for r in self.reasons
        ):
            raise ValueError(f"Invalid reasons for {self.role}")
        return self


# =============================================================================
# assembly_definition.yaml Schema
# =============================================================================


class ManufacturedPartEstimate(StrictContractModel):
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


class CotsPartEstimate(StrictContractModel):
    """Assembly estimate for a COTS part."""

    part_id: str
    manufacturer: str
    unit_cost_usd: float
    quantity: int
    source: str


class AssemblyPartConfig(StrictContractModel):
    """Configuration for a part in an assembly, including motion metadata."""

    dofs: list[str] = []
    control: MotorControl | None = None


class PartConfig(StrictContractModel):
    """Configuration for a part in an assembly, including motion metadata."""

    name: str
    config: AssemblyPartConfig


class JointEstimate(StrictContractModel):
    """Estimate for a joint in the assembly."""

    joint_id: str
    parts: list[str]
    type: str


class SubassemblyEstimate(StrictContractModel):
    """Estimate for a subassembly within the final assembly."""

    subassembly_id: str
    parts: list[PartConfig]
    joints: list[JointEstimate] = []


class CostTotals(StrictContractModel):
    """Total estimation for the design."""

    estimated_unit_cost_usd: float
    estimated_weight_g: float
    estimate_confidence: Literal["low", "medium", "high"]


class AssemblyUnits(StrictContractModel):
    """Units used in the estimation file."""

    length: str = "mm"
    volume: str = "mm3"
    mass: str = "g"
    currency: str = "USD"


class AssemblyConstraints(StrictContractModel):
    """Cap values from benchmark vs planner targets."""

    benchmark_max_unit_cost_usd: float
    benchmark_max_weight_g: float
    planner_target_max_unit_cost_usd: float
    planner_target_max_weight_g: float


# =============================================================================
# WP3 Assembly Electronics Section
# =============================================================================


class WireTerminal(StrictContractModel):
    """A terminal on an electronic component."""

    component: str
    terminal: str


class WireConfig(StrictContractModel):
    """Configuration for a physical wire in the assembly."""

    wire_id: str
    from_terminal: WireTerminal = Field(..., alias="from")
    to_terminal: WireTerminal = Field(..., alias="to")
    gauge_awg: int
    length_mm: float
    waypoints: list[tuple[float, float, float]] = []
    routed_in_3d: bool = False

    model_config = ConfigDict(populate_by_name=True, extra="forbid")

    @field_validator("waypoints", mode="before")
    @classmethod
    def coerce_waypoints(cls, v):
        if isinstance(v, list):
            return [tuple(pt) if isinstance(pt, list) else pt for pt in v]
        return v


class ElectronicComponent(StrictContractModel):
    """A component in the electronic circuit."""

    component_id: str
    type: ElectronicComponentType
    cots_part_id: str | None = None
    assembly_part_ref: str | None = None
    rated_voltage: float | None = None
    stall_current_a: float | None = None


class ElectronicsSection(StrictContractModel):
    """Electronics section of the assembly definition."""

    power_supply: PowerSupplyConfig
    wiring: list[WireConfig] = []
    components: list[ElectronicComponent] = []


class EnvironmentDrillOperation(StrictContractModel):
    """Planner-declared drill operation against a benchmark-owned fixture."""

    target_part_id: str
    hole_id: str
    diameter_mm: float
    depth_mm: float
    quantity: int = 1
    notes: str | None = None

    @model_validator(mode="after")
    def validate_contract(self) -> "EnvironmentDrillOperation":
        if self.diameter_mm <= 0:
            raise ValueError("diameter_mm must be > 0")
        if self.depth_mm <= 0:
            raise ValueError("depth_mm must be > 0")
        if self.quantity < 1:
            raise ValueError("quantity must be >= 1")
        return self


class AssemblyDefinition(StrictContractModel):
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
    environment_drill_operations: list[EnvironmentDrillOperation] = []
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
