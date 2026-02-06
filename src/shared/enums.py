"""Shared enum definitions for the Problemologist system.

All categorical data should use these enums instead of raw strings,
as mandated by the constitution's Type Safety & Schemas rules.
"""

from enum import StrEnum


class EpisodeStatus(StrEnum):
    """Status of an agent episode."""

    RUNNING = "running"
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


class ResponseStatus(StrEnum):
    """Standard API response status values."""

    OK = "ok"
    HEALTHY = "healthy"
    ACCEPTED = "accepted"
    COMPLETED = "completed"
    SUCCESS = "success"
    ERROR = "error"
