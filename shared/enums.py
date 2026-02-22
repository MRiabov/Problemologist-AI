"""Shared enum definitions for the Problemologist system.

All categorical data should use these enums instead of raw strings,
as mandated by the constitution's Type Safety & Schemas rules.
"""

from enum import StrEnum


class EpisodeStatus(StrEnum):
    """Status of an agent episode."""

    RUNNING = "running"
    PLANNED = "planned"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class SessionStatus(StrEnum):
    """Status of a benchmark generation session."""

    PLANNING = "planning"
    PLANNED = "planned"
    EXECUTING = "executing"
    VALIDATING = "validating"
    ACCEPTED = "accepted"
    REJECTED = "rejected"
    FAILED = "failed"


class AssetType(StrEnum):
    """Type of asset stored in S3."""

    VIDEO = "video"
    MJCF = "mjcf"
    IMAGE = "image"
    STEP = "step"
    STL = "stl"
    GLB = "glb"
    PYTHON = "python"
    OTHER = "other"


class ResponseStatus(StrEnum):
    """Standard API response status values."""

    OK = "ok"
    HEALTHY = "healthy"
    ACCEPTED = "accepted"
    COMPLETED = "completed"
    SUCCESS = "success"
    ERROR = "error"


class TraceType(StrEnum):
    """Type of observability trace."""

    TOOL_START = "tool_start"
    TOOL_END = "tool_end"
    LLM_START = "llm_start"
    LLM_END = "llm_end"
    LOG = "log"
    ERROR = "error"
    EVENT = "event"


class FailureReason(StrEnum):
    """Unified failure modes for physics and electronics simulation."""

    NONE = "NONE"
    TIMEOUT = "TIMEOUT"
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    FORBID_ZONE_HIT = "FORBID_ZONE_HIT"
    PART_BREAKAGE = "PART_BREAKAGE"
    MOTOR_OVERLOAD = "MOTOR_OVERLOAD"
    STABILITY_ISSUE = "STABILITY_ISSUE"
    PHYSICS_INSTABILITY = "PHYSICS_INSTABILITY"

    # Electronics failures
    SHORT_CIRCUIT = "SHORT_CIRCUIT"
    OVERCURRENT = "OVERCURRENT"
    WIRE_TORN = "WIRE_TORN"
    OPEN_CIRCUIT = "OPEN_CIRCUIT"

    # WP2 Fluids & Physics failures
    ASSET_GENERATION_FAILED = "ASSET_GENERATION_FAILED"
    FLUID_OBJECTIVE_FAILED = "FLUID_OBJECTIVE_FAILED"
    STRESS_OBJECTIVE_EXCEEDED = "STRESS_OBJECTIVE_EXCEEDED"
    ELECTRONICS_FLUID_DAMAGE = "ELECTRONICS_FLUID_DAMAGE"

    # Validation failures
    VALIDATION_FAILED = "VALIDATION_FAILED"
    MANUFACTURABILITY_FAILED = "MANUFACTURABILITY_FAILED"


# Alias for backward compatibility
SimulationFailureMode = FailureReason


class ElectronicComponentType(StrEnum):
    """Types of electronic components supported in the netlist."""

    MOTOR = "motor"
    POWER_SUPPLY = "power_supply"
    RELAY = "relay"
    SWITCH = "switch"
    CONNECTOR = "connector"


class ManufacturingMethod(StrEnum):
    """Supported manufacturing methods."""

    CNC = "cnc"
    THREE_DP = "3dp"
    INJECTION_MOLDING = "im"


class MotorControlMode(StrEnum):
    """Control modes for actuators."""

    CONSTANT = "constant"
    SINUSOIDAL = "sinusoidal"
    ON_OFF = "on_off"


class FluidShapeType(StrEnum):
    """Supported shapes for fluid volumes."""

    CYLINDER = "cylinder"
    BOX = "box"
    SPHERE = "sphere"


class FluidObjectiveType(StrEnum):
    """Types of fluid-related objectives."""

    FLUID_CONTAINMENT = "fluid_containment"
    FLOW_RATE = "flow_rate"


class FluidEvalAt(StrEnum):
    """When to evaluate fluid objectives."""

    END = "end"
    CONTINUOUS = "continuous"


class MovingPartType(StrEnum):
    """Types of moving parts in the assembly."""

    MOTOR = "motor"
    PASSIVE = "passive"


class ReviewDecision(StrEnum):
    """Decision values for reviewer nodes."""

    APPROVED = "approved"
    REJECTED = "rejected"
    REJECT_PLAN = "reject_plan"
    REJECT_CODE = "reject_code"
    CONFIRM_PLAN_REFUSAL = "confirm_plan_refusal"
    REJECT_PLAN_REFUSAL = "reject_plan_refusal"
