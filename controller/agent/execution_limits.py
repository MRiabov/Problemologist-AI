from __future__ import annotations

from datetime import UTC, datetime
from uuid import UUID

from pydantic import BaseModel, Field

from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.agents.config import (
    AgentExecutionPolicy,
    AgentsConfig,
    load_agents_config,
)
from shared.enums import AgentName
from shared.models.schemas import EpisodeMetadata


class EpisodeRuntimeCounters(BaseModel):
    elapsed_seconds: int = 0
    turn_count: int = 0
    granted_additional_turns: int = 0
    total_tokens_used: int = 0


class HardFailEvaluation(BaseModel):
    should_fail: bool = False
    code: str | None = None
    message: str | None = None
    policy: AgentExecutionPolicy
    counters: EpisodeRuntimeCounters = Field(default_factory=EpisodeRuntimeCounters)


class AgentHardFailError(RuntimeError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _load_agents_config() -> AgentsConfig:
    return load_agents_config()


def resolve_agent_execution_policy(agent_name: AgentName | str) -> AgentExecutionPolicy:
    config = _load_agents_config()
    normalized = (
        agent_name if isinstance(agent_name, AgentName) else AgentName(agent_name)
    )
    return config.execution.get_policy(normalized)


async def get_episode_runtime_counters(
    episode_id: str | UUID,
    turn_count: int | None = None,
) -> EpisodeRuntimeCounters:
    episode_uuid = UUID(str(episode_id))
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if not episode:
            return EpisodeRuntimeCounters(turn_count=int(turn_count or 0))

        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        additional = dict(metadata.additional_info or {})
        credit_usage = additional.get("credit_usage") or {}

        observed_turn_count = turn_count
        if observed_turn_count is None:
            observed_turn_count = int(additional.get("turn_count") or 0)

        granted_additional_turns = int(additional.get("granted_additional_turns") or 0)
        total_tokens_used = int(
            credit_usage.get("total_tokens")
            or credit_usage.get("total_tokens_used")
            or 0
        )

        execution_start_raw = additional.get("hard_fail_window_started_at")
        execution_start: datetime | None = None
        if isinstance(execution_start_raw, str):
            try:
                parsed = datetime.fromisoformat(execution_start_raw)
                execution_start = (
                    parsed.replace(tzinfo=UTC)
                    if parsed.tzinfo is None
                    else parsed.astimezone(UTC)
                )
            except Exception:
                execution_start = None

        created_at = episode.created_at
        now = datetime.now(UTC)
        if execution_start is not None:
            elapsed_seconds = max(0, int((now - execution_start).total_seconds()))
        elif created_at is None:
            elapsed_seconds = 0
        else:
            created_at_utc = (
                created_at.replace(tzinfo=UTC)
                if created_at.tzinfo is None
                else created_at.astimezone(UTC)
            )
            elapsed_seconds = max(0, int((now - created_at_utc).total_seconds()))

        return EpisodeRuntimeCounters(
            elapsed_seconds=elapsed_seconds,
            turn_count=max(0, int(observed_turn_count or 0)),
            granted_additional_turns=max(0, granted_additional_turns),
            total_tokens_used=max(0, total_tokens_used),
        )


async def evaluate_agent_hard_fail(
    *,
    agent_name: AgentName | str,
    episode_id: str | UUID | None,
    turn_count: int | None = None,
) -> HardFailEvaluation:
    policy = resolve_agent_execution_policy(agent_name)
    if not episode_id:
        return HardFailEvaluation(
            should_fail=True,
            code="missing_episode_id",
            message=(
                "Agent hard-fail: missing episode_id. "
                "Execution must be linked to an episode for quota enforcement."
            ),
            policy=policy,
            counters=EpisodeRuntimeCounters(turn_count=int(turn_count or 0)),
        )

    counters = await get_episode_runtime_counters(
        episode_id=episode_id, turn_count=turn_count
    )
    max_turn_budget = policy.max_turns + counters.granted_additional_turns

    if counters.elapsed_seconds >= policy.timeout_seconds:
        return HardFailEvaluation(
            should_fail=True,
            code="timeout",
            message=(
                f"Agent hard-fail: timeout reached ({counters.elapsed_seconds}s >= "
                f"{policy.timeout_seconds}s)."
            ),
            policy=policy,
            counters=counters,
        )

    if counters.turn_count >= max_turn_budget:
        return HardFailEvaluation(
            should_fail=True,
            code="max_turns",
            message=(
                f"Agent hard-fail: max turns reached ({counters.turn_count} >= "
                f"{max_turn_budget}). Use `additional_turns` in continue/confirm "
                "requests to continue."
            ),
            policy=policy,
            counters=counters,
        )

    if counters.total_tokens_used >= policy.max_total_tokens:
        return HardFailEvaluation(
            should_fail=True,
            code="credits_exceeded",
            message=(
                "Agent hard-fail: token credit limit reached "
                f"({counters.total_tokens_used} >= {policy.max_total_tokens})."
            ),
            policy=policy,
            counters=counters,
        )

    return HardFailEvaluation(policy=policy, counters=counters)


async def persist_episode_turn_count(
    episode_id: str | UUID,
    turn_count: int,
) -> None:
    episode_uuid = UUID(str(episode_id))
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if not episode:
            return
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        additional = dict(metadata.additional_info or {})
        additional["turn_count"] = max(0, int(turn_count))
        metadata.additional_info = additional
        episode.metadata_vars = metadata.model_dump()
        await db.commit()


async def mark_episode_execution_window_start(episode_id: str | UUID) -> None:
    episode_uuid = UUID(str(episode_id))
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if not episode:
            return
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        additional = dict(metadata.additional_info or {})
        additional["hard_fail_window_started_at"] = datetime.now(UTC).isoformat()
        metadata.additional_info = additional
        episode.metadata_vars = metadata.model_dump()
        await db.commit()


async def grant_episode_additional_turns(
    episode_id: str | UUID,
    additional_turns: int,
) -> int:
    episode_uuid = UUID(str(episode_id))
    amount = max(0, int(additional_turns))
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if not episode:
            return 0
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        additional = dict(metadata.additional_info or {})
        current = int(additional.get("granted_additional_turns") or 0)
        additional["granted_additional_turns"] = current + amount
        metadata.additional_info = additional
        episode.metadata_vars = metadata.model_dump()
        await db.commit()
        return int(additional["granted_additional_turns"])


async def accumulate_episode_credit_usage(
    episode_id: str | UUID,
    *,
    input_tokens: int | None,
    output_tokens: int | None,
    total_tokens: int | None,
    cost_usd: float | None = None,
) -> None:
    episode_uuid = UUID(str(episode_id))
    in_tokens = max(0, int(input_tokens or 0))
    out_tokens = max(0, int(output_tokens or 0))
    total = (
        max(0, int(total_tokens))
        if total_tokens is not None
        else in_tokens + out_tokens
    )
    normalized_cost_usd = (
        max(0.0, float(cost_usd)) if isinstance(cost_usd, int | float) else 0.0
    )

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if not episode:
            return

        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        additional = dict(metadata.additional_info or {})
        current_usage = dict(additional.get("credit_usage") or {})
        current_input = int(current_usage.get("input_tokens") or 0)
        current_output = int(current_usage.get("output_tokens") or 0)
        current_total = int(current_usage.get("total_tokens") or 0)

        current_usage["input_tokens"] = current_input + in_tokens
        current_usage["output_tokens"] = current_output + out_tokens
        current_usage["total_tokens"] = current_total + total
        current_cost = float(current_usage.get("total_cost_usd") or 0.0)
        current_usage["total_cost_usd"] = current_cost + normalized_cost_usd
        additional["credit_usage"] = current_usage
        metadata.additional_info = additional
        episode.metadata_vars = metadata.model_dump()
        await db.commit()
