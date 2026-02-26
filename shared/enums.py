"""Shared enum definitions for the Problemologist system.

All categorical data should use these enums instead of raw strings,
as mandated by the constitution's Type Safety & Schemas rules.
"""

from enum import StrEnum


class UppercaseStrEnum(StrEnum):
    """Uppercase enum values with backward-compatible case-insensitive matching."""

    @classmethod
    def _missing_(cls, value):
        if isinstance(value, str):
            normalized = value.upper()
            for member in cls:
                if member.value == normalized:
                    return member
        return None

    def __eq__(self, other):
        if isinstance(other, str):
            return self.value.lower() == other.lower()
        return super().__eq__(other)

    __hash__ = str.__hash__


class EpisodeStatus(StrEnum):
    """Status of an agent episode."""

    RUNNING = "RUNNING"
    PLANNED = "PLANNED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class SessionStatus(StrEnum):
    """Status of a benchmark generation session."""

    PLANNING = "PLANNING"
    PLANNED = "PLANNED"
    EXECUTING = "EXECUTING"
    VALIDATING = "VALIDATING"
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    FAILED = "FAILED"


class AssetType(StrEnum):
    """Type of asset stored in S3."""

    VIDEO = "VIDEO"
    MJCF = "MJCF"
    IMAGE = "IMAGE"
    STEP = "STEP"
    STL = "STL"
    GLB = "GLB"
    PYTHON = "PYTHON"
    OTHER = "OTHER"
    CIRCUIT_DATA = "CIRCUIT_DATA"
    TIMELINE = "TIMELINE"
    MARKDOWN = "MARKDOWN"
    LOG = "LOG"
    ERROR = "ERROR"


class ResponseStatus(UppercaseStrEnum):
    """Standard API response status values."""

    OK = "OK"
    HEALTHY = "HEALTHY"
    ACCEPTED = "ACCEPTED"
    COMPLETED = "COMPLETED"
    SUCCESS = "SUCCESS"
    ERROR = "ERROR"


class TraceType(StrEnum):
    """Type of observability trace."""

    TOOL_START = "TOOL_START"
    TOOL_END = "TOOL_END"
    LLM_START = "LLM_START"
    LLM_END = "LLM_END"
    LOG = "LOG"
    ERROR = "ERROR"
    EVENT = "EVENT"


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
    VERIFICATION_ERROR = "VERIFICATION_ERROR"


# Alias for backward compatibility
SimulationFailureMode = FailureReason


class ElectronicComponentType(StrEnum):
    """Types of electronic components supported in the netlist."""

    MOTOR = "MOTOR"
    POWER_SUPPLY = "POWER_SUPPLY"
    RELAY = "RELAY"
    SWITCH = "SWITCH"
    CONNECTOR = "CONNECTOR"
    LOGIC_BOARD = "LOGIC_BOARD"


class ManufacturingMethod(StrEnum):
    """Supported manufacturing methods."""

    CNC = "CNC"
    THREE_DP = "3DP"
    INJECTION_MOLDING = "IM"


class MotorControlMode(StrEnum):
    """Control modes for actuators."""

    CONSTANT = "CONSTANT"
    SINUSOIDAL = "SINUSOIDAL"
    ON_OFF = "ON_OFF"


class FluidShapeType(StrEnum):
    """Supported shapes for fluid volumes."""

    CYLINDER = "CYLINDER"
    BOX = "BOX"
    SPHERE = "SPHERE"


class FluidObjectiveType(StrEnum):
    """Types of fluid-related objectives."""

    FLUID_CONTAINMENT = "FLUID_CONTAINMENT"
    FLOW_RATE = "FLOW_RATE"


class FluidEvalAt(StrEnum):
    """When to evaluate fluid objectives."""

    END = "END"
    CONTINUOUS = "CONTINUOUS"


class MovingPartType(StrEnum):
    """Types of moving parts in the assembly."""

    MOTOR = "MOTOR"
    PASSIVE = "PASSIVE"


class ReviewDecision(UppercaseStrEnum):
    """Decision values for reviewer nodes."""

    APPROVED = "APPROVED"
    REJECTED = "REJECTED"
    REJECT_PLAN = "REJECT_PLAN"
    REJECT_CODE = "REJECT_CODE"
    CONFIRM_PLAN_REFUSAL = "CONFIRM_PLAN_REFUSAL"
    REJECT_PLAN_REFUSAL = "REJECT_PLAN_REFUSAL"
