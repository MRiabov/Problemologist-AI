import uuid

from .integration import (
    apply_integration_test_metadata as apply_integration_test_metadata,
)
from .integration import infer_integration_test_id as infer_integration_test_id


def get_episode_id(session_id: str | uuid.UUID) -> uuid.UUID:
    """
    Derive a deterministic UUID from a session_id string, or return the UUID if already one.

    This ensures that different components (API, Agent Nodes, Asset Sync) that might only
    have the session_id string can always re-derive the correct episode_id.
    """
    if isinstance(session_id, uuid.UUID):
        return session_id

    try:
        # 1. Try parsing as standard UUID
        return uuid.UUID(session_id)
    except ValueError:
        # 2. Handle prefixes like INT-xxx-, sim-, sess- etc.
        # If it's a known non-UUID format, we use uuid5 to derive it deterministically.
        # This is critical for integration tests and benchmarks.
        return uuid.uuid5(uuid.NAMESPACE_DNS, session_id)
