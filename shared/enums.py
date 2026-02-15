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
    """Specific failure modes for physics and electronics simulation."""

    # WP3 Electronics failures
    SHORT_CIRCUIT = "FAILED_SHORT_CIRCUIT"
    OVERCURRENT_SUPPLY = "FAILED_OVERCURRENT_SUPPLY"
    OVERCURRENT_WIRE = "FAILED_OVERCURRENT_WIRE"
    OPEN_CIRCUIT = "FAILED_OPEN_CIRCUIT"
    WIRE_TORN = "FAILED_WIRE_TORN"

    # WP2 Fluids & Physics failures
    ASSET_GENERATION = "FAILED_ASSET_GENERATION"
    FLUID_OBJECTIVE_FAILED = "FAILED_FLUID_OBJECTIVE"
    PART_BREAKAGE = "FAILED_PART_BREAKAGE"
    STRESS_OBJECTIVE_EXCEEDED = "FAILED_STRESS_OBJECTIVE"
    ELECTRONICS_FLUID_DAMAGE = "FAILED_ELECTRONICS_FLUID_DAMAGE"
