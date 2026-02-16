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


from shared.simulation.schemas import SimulationFailureMode


class ElectricalFailureType(StrEnum):
    """Standard electrical failure categories."""

    SHORT_CIRCUIT = "short_circuit"
    OVERCURRENT = "overcurrent"
    WIRE_TORN = "wire_torn"
    OPEN_CIRCUIT = "open_circuit"


class ElectronicComponentType(StrEnum):
    """Types of electronic components supported in the netlist."""

    MOTOR = "motor"
    POWER_SUPPLY = "power_supply"
    RELAY = "relay"
    SWITCH = "switch"
    CONNECTOR = "connector"
