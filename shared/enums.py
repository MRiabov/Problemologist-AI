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


class SimulationFailureMode(StrEnum):
    """Unified failure modes for physics and electronics simulation."""

    NONE = "none"
    TIMEOUT = "timeout"
    OUT_OF_BOUNDS = "out_of_bounds"
    FORBID_ZONE_HIT = "forbid_zone_hit"
    PART_BREAKAGE = "part_breakage"
    STABILITY_ISSUE = "stability_issue"
    PHYSICS_INSTABILITY = "physics_instability"

    # Electronics failures
    SHORT_CIRCUIT = "short_circuit"
    OVERCURRENT = "overcurrent"
    WIRE_TORN = "wire_torn"
    OPEN_CIRCUIT = "open_circuit"

    # WP2 Fluids & Physics failures
    ASSET_GENERATION_FAILED = "asset_generation_failed"
    FLUID_OBJECTIVE_FAILED = "fluid_objective_failed"
    STRESS_OBJECTIVE_EXCEEDED = "stress_objective_exceeded"
    ELECTRONICS_FLUID_DAMAGE = "electronics_fluid_damage"

    # Validation failures
    VALIDATION_FAILED = "validation_failed"
    MANUFACTURABILITY_FAILED = "manufacturability_failed"


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
