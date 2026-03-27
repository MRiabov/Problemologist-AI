import uuid

from .identity import EpisodeIdentity, resolve_episode_id
from .integration import (
    apply_integration_test_metadata as apply_integration_test_metadata,
)
from .integration import infer_integration_test_id as infer_integration_test_id


def get_episode_id(session_id: str | uuid.UUID) -> uuid.UUID:
    """Backward-compatible wrapper around :func:`resolve_episode_id`."""
    return resolve_episode_id(session_id)


__all__ = [
    "EpisodeIdentity",
    "apply_integration_test_metadata",
    "get_episode_id",
    "infer_integration_test_id",
    "resolve_episode_id",
]
