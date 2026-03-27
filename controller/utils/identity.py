from __future__ import annotations

import uuid

from pydantic import BaseModel, ConfigDict


def resolve_episode_id(value: str | uuid.UUID) -> uuid.UUID:
    """
    Resolve the canonical episode UUID from a session/episode identifier.

    Legacy callers often only have a session identifier. When that identifier is
    already a UUID, we preserve it. Otherwise we derive a deterministic UUID so
    the controller, worker, and observability layers can join on one stable key.
    """
    if isinstance(value, uuid.UUID):
        return value

    try:
        return uuid.UUID(value)
    except ValueError:
        return uuid.uuid5(uuid.NAMESPACE_DNS, value)


class EpisodeIdentity(BaseModel):
    """Normalized session/episode identity for controller-side workflows."""

    session_id: str
    episode_id: uuid.UUID

    model_config = ConfigDict(extra="forbid", frozen=True)

    @classmethod
    def from_context(
        cls,
        session_id: str | uuid.UUID,
        episode_id: str | uuid.UUID | None = None,
    ) -> "EpisodeIdentity":
        normalized_session_id = str(session_id)
        resolved_episode_id = resolve_episode_id(episode_id or session_id)
        return cls(
            session_id=normalized_session_id,
            episode_id=resolved_episode_id,
        )
