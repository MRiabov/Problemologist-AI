from __future__ import annotations

import asyncio
import uuid
from collections.abc import Mapping, Sequence
from enum import StrEnum
from typing import Any, Protocol

import httpx
from pydantic import BaseModel, Field, model_validator

from controller.agent.benchmark_handover_validation import (
    extract_benchmark_refusal_reason,
    extract_custom_objectives_from_state,
    validate_benchmark_planner_handoff_artifacts,
)
from controller.agent.config import settings as agent_settings
from controller.agent.handover_constants import (
    BENCHMARK_CODER_HANDOVER_CHECK,
    BENCHMARK_PLAN_REVIEW_MANIFEST,
    BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK,
    BENCHMARK_PLANNER_HANDOFF_ARTIFACTS,
    BENCHMARK_REVIEWER_HANDOVER_CHECK,
    ELECTRONICS_REVIEWER_HANDOVER_CHECK,
    ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS,
    ENGINEER_BENCHMARK_HANDOVER_CHECK,
    ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK,
    ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK,
    ENGINEER_PLANNER_HANDOFF_ARTIFACTS,
    ENGINEERING_EXECUTION_REVIEW_MANIFEST,
    ENGINEERING_PLAN_REVIEW_MANIFEST,
    REVIEWER_HANDOFF_ARTIFACTS,
    SCHEMA_BACKED_HANDOFF_PATHS,
)
from controller.agent.render_validation import validate_render_images_non_black
from controller.agent.review_handover import (
    collect_plan_reviewer_handover_evidence,
    validate_approved_benchmark_bundle,
    validate_plan_reviewer_handover,
    validate_planner_artifacts_cross_contract,
    validate_reviewer_handover,
)
from controller.clients.worker import WorkerClient
from controller.config.settings import settings as controller_settings
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.enums import AgentName, EntryFailureDisposition, EntryValidationSource
from shared.models.schemas import (
    AssemblyDefinition,
    BenchmarkDefinition,
    EpisodeMetadata,
)
from shared.models.simulation import SimulationResult
from shared.simulation.schemas import CustomObjectives
from shared.workers.markdown_validator import validate_todo_md
from shared.workers.schema import (
    PlanReviewManifest,
    ReviewerStage,
    ReviewManifest,
    ValidationResultRecord,
)
from worker_heavy.utils.dfm import load_planner_manufacturing_config_from_text
from worker_heavy.utils.file_validation import (
    validate_assembly_definition_yaml,
    validate_benchmark_assembly_motion_contract,
    validate_benchmark_definition_yaml,
    validate_plan_md_structure,
    validate_plan_refusal,
    validate_planner_handoff_cross_contract,
)

REASON_OK = "ok"
REASON_STATE_INVALID = "state_invalid"
REASON_MISSING_ARTIFACT = "missing_artifact"
REASON_HANDOVER_INVALID = "handover_invalid"
REASON_REVIEWER_ENTRY_BLOCKED = "reviewer_entry_blocked"
REASON_POLICY_INVALID = "policy_invalid"
REASON_NO_PREVIOUS_NODE = "no_previous_node"
REASON_CUSTOM_CHECK_FAILED = "custom_check_failed"


class ValidationGraph(StrEnum):
    ENGINEER = "engineer"
    BENCHMARK = "benchmark"


# Static and deterministic by contract. If a node has no previous node, reroute
# is impossible and callers must fail closed.
ENGINEER_PREVIOUS_NODE_MAP: Mapping[AgentName, AgentName | None] = {
    AgentName.ENGINEER_PLANNER: None,
    AgentName.ELECTRONICS_PLANNER: AgentName.ENGINEER_PLANNER,
    AgentName.ENGINEER_PLAN_REVIEWER: AgentName.ELECTRONICS_PLANNER,
    AgentName.ENGINEER_CODER: AgentName.ENGINEER_PLAN_REVIEWER,
    AgentName.ELECTRONICS_REVIEWER: AgentName.ENGINEER_CODER,
    # Execution review failures should route back to coder so latest-revision
    # handover artifacts can be regenerated before another reviewer entry.
    AgentName.ENGINEER_EXECUTION_REVIEWER: AgentName.ENGINEER_CODER,
    AgentName.COTS_SEARCH: AgentName.ENGINEER_PLANNER,
    AgentName.SKILL_AGENT: AgentName.ENGINEER_EXECUTION_REVIEWER,
    AgentName.JOURNALLING_AGENT: AgentName.ENGINEER_PLAN_REVIEWER,
    AgentName.STEER: None,
}

BENCHMARK_PREVIOUS_NODE_MAP: Mapping[AgentName, AgentName | None] = {
    AgentName.BENCHMARK_PLANNER: None,
    AgentName.BENCHMARK_PLAN_REVIEWER: AgentName.BENCHMARK_PLANNER,
    AgentName.BENCHMARK_CODER: AgentName.BENCHMARK_PLAN_REVIEWER,
    AgentName.BENCHMARK_REVIEWER: AgentName.BENCHMARK_CODER,
    AgentName.COTS_SEARCH: AgentName.BENCHMARK_PLANNER,
    AgentName.SKILL_AGENT: AgentName.BENCHMARK_REVIEWER,
    AgentName.JOURNALLING_AGENT: AgentName.BENCHMARK_REVIEWER,
    AgentName.STEER: None,
}

PREVIOUS_NODE_MAPS: Mapping[ValidationGraph, Mapping[AgentName, AgentName | None]] = {
    ValidationGraph.ENGINEER: ENGINEER_PREVIOUS_NODE_MAP,
    ValidationGraph.BENCHMARK: BENCHMARK_PREVIOUS_NODE_MAP,
}

_TRANSIENT_BUSY_BASE_DELAY_SECONDS = 1.0
_TRANSIENT_BUSY_MAX_WAIT_SECONDS = 180.0


class NodeEntryValidationError(BaseModel):
    code: str
    message: str
    source: EntryValidationSource
    artifact_path: str | None = None


class NodeEntryValidationResult(BaseModel):
    ok: bool
    target_node: AgentName
    disposition: EntryFailureDisposition
    errors: list[NodeEntryValidationError] = Field(default_factory=list)
    reroute_target: AgentName | None = None
    reason_code: str = REASON_OK

    @model_validator(mode="after")
    def validate_contract(self) -> NodeEntryValidationResult:
        if self.ok:
            if self.disposition != EntryFailureDisposition.ALLOW:
                msg = "ok=true requires disposition=allow"
                raise ValueError(msg)
            if self.errors:
                raise ValueError("ok=true requires errors=[]")
            if self.reroute_target is not None:
                raise ValueError("ok=true requires reroute_target=None")
        else:
            if not self.errors:
                raise ValueError("ok=false requires at least one error")
            if (
                self.disposition == EntryFailureDisposition.REROUTE_PREVIOUS
                and self.reroute_target is None
            ):
                raise ValueError("disposition=reroute_previous requires reroute_target")
        return self


class NodeEntryContract(BaseModel):
    node: AgentName
    required_state_fields: list[str] = Field(default_factory=list)
    required_artifacts: list[str] = Field(default_factory=list)
    custom_check: str | None = None
    integration_failure_policy: EntryFailureDisposition = (
        EntryFailureDisposition.FAIL_FAST
    )


def build_benchmark_node_contracts() -> dict[AgentName, NodeEntryContract]:
    # Scope boundary: contracts apply only to first-class graph transitions.
    # Tool-invoked helper subagents are explicitly out of scope for this registry.
    return {
        AgentName.BENCHMARK_PLANNER: NodeEntryContract(
            node=AgentName.BENCHMARK_PLANNER,
            required_state_fields=["session", "episode_id"],
        ),
        AgentName.BENCHMARK_PLAN_REVIEWER: NodeEntryContract(
            node=AgentName.BENCHMARK_PLAN_REVIEWER,
            required_state_fields=["session", "episode_id"],
            required_artifacts=list(BENCHMARK_PLANNER_HANDOFF_ARTIFACTS),
            custom_check=BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK,
        ),
        AgentName.BENCHMARK_CODER: NodeEntryContract(
            node=AgentName.BENCHMARK_CODER,
            required_state_fields=["session", "episode_id"],
            required_artifacts=list(BENCHMARK_PLANNER_HANDOFF_ARTIFACTS),
            custom_check=BENCHMARK_CODER_HANDOVER_CHECK,
        ),
        AgentName.BENCHMARK_REVIEWER: NodeEntryContract(
            node=AgentName.BENCHMARK_REVIEWER,
            required_state_fields=["session", "episode_id"],
            custom_check=BENCHMARK_REVIEWER_HANDOVER_CHECK,
        ),
        AgentName.COTS_SEARCH: NodeEntryContract(
            node=AgentName.COTS_SEARCH,
            required_state_fields=["session", "episode_id"],
        ),
        AgentName.SKILL_AGENT: NodeEntryContract(
            node=AgentName.SKILL_AGENT,
            required_state_fields=["session", "episode_id"],
        ),
        AgentName.JOURNALLING_AGENT: NodeEntryContract(
            node=AgentName.JOURNALLING_AGENT,
            required_state_fields=["session", "episode_id"],
        ),
        AgentName.STEER: NodeEntryContract(
            node=AgentName.STEER,
            required_state_fields=["session", "episode_id"],
        ),
    }


def build_engineer_node_contracts() -> dict[AgentName, NodeEntryContract]:
    # Scope boundary: only first-class orchestration nodes are entry-guarded.
    return {
        AgentName.ENGINEER_PLANNER: NodeEntryContract(
            node=AgentName.ENGINEER_PLANNER,
            required_state_fields=["task", "episode_id"],
            required_artifacts=list(ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS),
            custom_check=ENGINEER_BENCHMARK_HANDOVER_CHECK,
        ),
        AgentName.ELECTRONICS_PLANNER: NodeEntryContract(
            node=AgentName.ELECTRONICS_PLANNER,
            required_state_fields=["task", "episode_id"],
            required_artifacts=list(ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS),
            custom_check=ENGINEER_BENCHMARK_HANDOVER_CHECK,
        ),
        AgentName.ENGINEER_PLAN_REVIEWER: NodeEntryContract(
            node=AgentName.ENGINEER_PLAN_REVIEWER,
            required_state_fields=["episode_id"],
            custom_check=ENGINEER_PLAN_REVIEWER_HANDOVER_CHECK,
        ),
        AgentName.ENGINEER_CODER: NodeEntryContract(
            node=AgentName.ENGINEER_CODER,
            required_state_fields=["episode_id"],
            required_artifacts=list(ENGINEER_PLANNER_HANDOFF_ARTIFACTS),
        ),
        AgentName.ELECTRONICS_REVIEWER: NodeEntryContract(
            node=AgentName.ELECTRONICS_REVIEWER,
            required_state_fields=["episode_id"],
            custom_check=ELECTRONICS_REVIEWER_HANDOVER_CHECK,
        ),
        AgentName.ENGINEER_EXECUTION_REVIEWER: NodeEntryContract(
            node=AgentName.ENGINEER_EXECUTION_REVIEWER,
            required_state_fields=["episode_id"],
            custom_check=ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK,
        ),
        AgentName.COTS_SEARCH: NodeEntryContract(
            node=AgentName.COTS_SEARCH,
            required_state_fields=["task", "episode_id"],
        ),
        AgentName.SKILL_AGENT: NodeEntryContract(
            node=AgentName.SKILL_AGENT,
            required_state_fields=["episode_id"],
        ),
        AgentName.JOURNALLING_AGENT: NodeEntryContract(
            node=AgentName.JOURNALLING_AGENT,
            required_state_fields=["episode_id"],
        ),
        AgentName.STEER: NodeEntryContract(
            node=AgentName.STEER,
            required_state_fields=["episode_id"],
        ),
    }


async def reviewer_handover_custom_check_from_session_id(
    *,
    session_id: str | None,
    reviewer_label: str,
    manifest_path: str,
    expected_stage: ReviewerStage,
    agent_role: AgentName | None = None,
) -> list[NodeEntryValidationError]:
    normalized_session_id = (session_id or "").strip()
    if not normalized_session_id:
        return [
            NodeEntryValidationError(
                code=REASON_REVIEWER_ENTRY_BLOCKED,
                message=(
                    f"{reviewer_label} reviewer handover check failed: "
                    "missing session_id."
                ),
                source=EntryValidationSource.HANDOVER,
            )
        ]

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=normalized_session_id,
        agent_role=agent_role or AgentName.ENGINEER_CODER,
    )
    try:
        handover_error = await validate_reviewer_handover(
            client,
            manifest_path=manifest_path,
            expected_stage=expected_stage,
            require_git_revision=True,
        )
    except Exception as exc:
        handover_error = f"reviewer handover validation exception: {exc}"
    finally:
        await client.aclose()

    if handover_error is None:
        return []

    return [
        NodeEntryValidationError(
            code=REASON_REVIEWER_ENTRY_BLOCKED,
            message=f"{reviewer_label} reviewer entry blocked: {handover_error}",
            source=EntryValidationSource.HANDOVER,
            artifact_path=manifest_path,
        )
    ]


async def benchmark_coder_handover_custom_check_from_session_id(
    *,
    session_id: str | None,
    custom_objectives: CustomObjectives | None = None,
) -> list[NodeEntryValidationError]:
    normalized_session_id = (session_id or "").strip()
    if not normalized_session_id:
        return [
            NodeEntryValidationError(
                code=REASON_HANDOVER_INVALID,
                message="Benchmark coder handover check failed: missing session_id.",
                source=EntryValidationSource.HANDOVER,
            )
        ]

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=normalized_session_id,
        agent_role=AgentName.BENCHMARK_CODER,
    )
    try:
        handoff_errors = await validate_benchmark_planner_handoff_artifacts(
            client,
            custom_objectives=custom_objectives,
        )
    except Exception as exc:
        handoff_errors = [f"benchmark planner handoff validation exception: {exc}"]
    finally:
        await client.aclose()

    return _benchmark_validation_errors(
        messages=handoff_errors,
        message_prefix="Benchmark coder entry blocked: ",
        artifact_path=None,
    )


async def benchmark_plan_reviewer_handover_custom_check_from_session_id(
    *,
    session_id: str | None,
    episode_id: str | None = None,
) -> list[NodeEntryValidationError]:
    normalized_session_id = (session_id or "").strip()
    if not normalized_session_id:
        return [
            NodeEntryValidationError(
                code=REASON_REVIEWER_ENTRY_BLOCKED,
                message=(
                    "Benchmark plan reviewer handover check failed: missing session_id."
                ),
                source=EntryValidationSource.HANDOVER,
                artifact_path=BENCHMARK_PLAN_REVIEW_MANIFEST,
            )
        ]

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=normalized_session_id,
        agent_role=AgentName.BENCHMARK_PLAN_REVIEWER,
    )
    evidence = None
    try:
        evidence, _ = await collect_plan_reviewer_handover_evidence(
            client,
            manifest_path=BENCHMARK_PLAN_REVIEW_MANIFEST,
            expected_stage=AgentName.BENCHMARK_PLAN_REVIEWER,
            episode_id=episode_id,
        )
        handover_error = await validate_plan_reviewer_handover(
            client,
            manifest_path=BENCHMARK_PLAN_REVIEW_MANIFEST,
            expected_stage=AgentName.BENCHMARK_PLAN_REVIEWER,
        )
    except Exception as exc:
        handover_error = f"plan reviewer handover validation exception: {exc}"
    finally:
        await client.aclose()

    if handover_error is None:
        return []

    if evidence is not None:
        evidence_bits = [
            f"revision={evidence.review_manifest_revision or 'unknown'}",
            f"renders={evidence.render_count}",
        ]
        if evidence.refusal_reason is not None:
            evidence_bits.append(f"refusal_reason={evidence.refusal_reason.value}")
        if evidence.deterministic_errors:
            evidence_bits.append(
                "deterministic_errors=" + " | ".join(evidence.deterministic_errors[:3])
            )
        handover_error = f"{handover_error} | evidence: {'; '.join(evidence_bits)}"

    return _benchmark_validation_errors(
        messages=[handover_error],
        message_prefix="Benchmark plan reviewer entry blocked: ",
        artifact_path=BENCHMARK_PLAN_REVIEW_MANIFEST,
        default_code=REASON_REVIEWER_ENTRY_BLOCKED,
    )


async def benchmark_plan_reviewer_handover_custom_check(
    *,
    contract: NodeEntryContract,  # noqa: ARG001
    state: BaseModel | Mapping[str, Any],
) -> list[NodeEntryValidationError]:
    worker_session_id = _get_state_value(state, "worker_session_id")
    session = _get_state_value(state, "session")
    if isinstance(session, Mapping):
        session_id = session.get("session_id")
    else:
        session_id = getattr(session, "session_id", None)
    return await benchmark_plan_reviewer_handover_custom_check_from_session_id(
        session_id=str(worker_session_id or session_id)
        if worker_session_id or session_id
        else None,
        episode_id=str(_get_state_value(state, "episode_id"))
        if _get_state_value(state, "episode_id")
        else None,
    )


async def benchmark_coder_handover_custom_check(
    *,
    contract: NodeEntryContract,  # noqa: ARG001
    state: BaseModel | Mapping[str, Any],
) -> list[NodeEntryValidationError]:
    worker_session_id = _get_state_value(state, "worker_session_id")
    session = _get_state_value(state, "session")
    if isinstance(session, Mapping):
        session_id = session.get("session_id")
    else:
        session_id = getattr(session, "session_id", None)
    return await benchmark_coder_handover_custom_check_from_session_id(
        session_id=str(worker_session_id or session_id)
        if worker_session_id or session_id
        else None,
        custom_objectives=extract_custom_objectives_from_state(state),
    )


async def engineer_benchmark_handover_custom_check(
    *,
    contract: NodeEntryContract,  # noqa: ARG001
    state: BaseModel | Mapping[str, Any],
) -> list[NodeEntryValidationError]:
    worker_session_id = _get_state_value(
        state, "worker_session_id"
    ) or _get_state_value(state, "session_id")
    episode_id = _get_state_value(state, "episode_id")
    if not worker_session_id:
        return [
            NodeEntryValidationError(
                code=REASON_HANDOVER_INVALID,
                message=(
                    "Engineer benchmark handover check failed: missing session_id."
                ),
                source=EntryValidationSource.HANDOVER,
                artifact_path="benchmark_definition.yaml",
            )
        ]
    if not episode_id:
        return [
            NodeEntryValidationError(
                code=REASON_STATE_INVALID,
                message=(
                    "Engineer benchmark handover check failed: missing episode_id."
                ),
                source=EntryValidationSource.STATE,
            )
        ]

    try:
        episode_uuid = uuid.UUID(str(episode_id).strip())
    except Exception:
        return [
            NodeEntryValidationError(
                code=REASON_STATE_INVALID,
                message=(
                    "Engineer benchmark handover check failed: invalid episode_id."
                ),
                source=EntryValidationSource.STATE,
            )
        ]

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_uuid)
        if episode is None:
            return [
                NodeEntryValidationError(
                    code=REASON_HANDOVER_INVALID,
                    message=(
                        "Engineer benchmark handover check failed: episode not found."
                    ),
                    source=EntryValidationSource.HANDOVER,
                    artifact_path="benchmark_definition.yaml",
                )
            ]
        metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
        benchmark_episode_id = (metadata.benchmark_id or "").strip()

    if not benchmark_episode_id:
        return []

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=str(worker_session_id),
        agent_role=AgentName.ENGINEER_PLANNER,
    )
    try:
        bundle, bundle_error = await validate_approved_benchmark_bundle(
            client,
            benchmark_episode_id=benchmark_episode_id,
        )
    except Exception as exc:
        bundle = None
        bundle_error = f"approved benchmark bundle validation exception: {exc}"
    finally:
        await client.aclose()

    if bundle_error is None and bundle is not None:
        return []

    return [
        NodeEntryValidationError(
            code=REASON_HANDOVER_INVALID,
            message=(
                "Engineer benchmark entry blocked: "
                f"{bundle_error or 'approved benchmark bundle invalid.'}"
            ),
            source=EntryValidationSource.HANDOVER,
            artifact_path="benchmark_definition.yaml",
        )
    ]


async def plan_reviewer_handover_custom_check_from_session_id(
    *,
    session_id: str | None,
) -> list[NodeEntryValidationError]:
    normalized_session_id = (session_id or "").strip()
    if not normalized_session_id:
        return [
            NodeEntryValidationError(
                code=REASON_REVIEWER_ENTRY_BLOCKED,
                message="Plan reviewer handover check failed: missing session_id.",
                source=EntryValidationSource.HANDOVER,
            )
        ]

    client = WorkerClient(
        base_url=controller_settings.worker_light_url,
        heavy_url=controller_settings.worker_heavy_url,
        session_id=normalized_session_id,
        agent_role=AgentName.ENGINEER_PLAN_REVIEWER,
    )
    try:
        handover_error = await validate_plan_reviewer_handover(
            client,
            manifest_path=ENGINEERING_PLAN_REVIEW_MANIFEST,
        )
    except Exception as exc:
        handover_error = f"plan reviewer handover validation exception: {exc}"
    finally:
        await client.aclose()

    if handover_error is None:
        return []

    return [
        NodeEntryValidationError(
            code=REASON_REVIEWER_ENTRY_BLOCKED,
            message=f"Plan reviewer entry blocked: {handover_error}",
            source=EntryValidationSource.HANDOVER,
            artifact_path=ENGINEERING_PLAN_REVIEW_MANIFEST,
        )
    ]


async def _materialize_reviewer_handover(
    client: WorkerClient,
    *,
    reviewer_stage: ReviewerStage = "engineering_execution_reviewer",
    episode_id: str | None = None,
) -> str | None:
    if reviewer_stage == "engineering_execution_reviewer":
        try:
            existing_handover_error = await validate_reviewer_handover(
                client,
                manifest_path=ENGINEERING_EXECUTION_REVIEW_MANIFEST,
                expected_stage=reviewer_stage,
            )
        except Exception:
            existing_handover_error = "unknown reviewer handover validation error"
        else:
            if existing_handover_error is None:
                return None

    async def _run_with_transient_busy_retry(operation_name: str, coro_factory):
        deadline = asyncio.get_running_loop().time() + _TRANSIENT_BUSY_MAX_WAIT_SECONDS
        last_exc: Exception | None = None
        attempt = 0
        while True:
            try:
                return await coro_factory()
            except httpx.HTTPStatusError as exc:
                status_code = exc.response.status_code if exc.response else None
                if status_code != 503:
                    raise
                last_exc = exc
                if asyncio.get_running_loop().time() >= deadline:
                    raise
                while asyncio.get_running_loop().time() < deadline:
                    try:
                        if await client.heavy_ready():
                            break
                    except Exception:
                        pass
                    await asyncio.sleep(1.0)
                else:
                    raise
            except (httpx.ConnectError, httpx.ReadTimeout) as exc:
                if asyncio.get_running_loop().time() >= deadline:
                    raise
                last_exc = exc
                await asyncio.sleep(
                    min(
                        _TRANSIENT_BUSY_BASE_DELAY_SECONDS * float(attempt + 1),
                        5.0,
                    )
                )
            except Exception:
                raise
            attempt += 1

        if last_exc is not None:
            raise last_exc
        raise RuntimeError(f"{operation_name} failed without exception")

    if not await client.exists("script.py"):
        return "script.py missing; cannot materialize review handover."

    try:
        validate_result = await _run_with_transient_busy_retry(
            "validate",
            lambda: client.validate("script.py"),
        )
    except Exception as exc:
        return f"validate failed while materializing handover: {exc}"
    if not validate_result.success:
        return (
            "validate failed while materializing handover: "
            f"{validate_result.message or 'unknown validation failure'}"
        )

    try:
        simulate_result = await _run_with_transient_busy_retry(
            "simulate",
            lambda: client.simulate("script.py"),
        )
    except Exception as exc:
        return f"simulate failed while materializing handover: {exc}"
    if not simulate_result.success:
        return (
            "simulate failed while materializing handover: "
            f"{simulate_result.message or 'unknown simulation failure'}"
        )

    try:
        verify_result = await _run_with_transient_busy_retry(
            "verify",
            lambda: client.verify(
                "script.py",
                num_scenes=1 if controller_settings.is_integration_test else None,
                duration=1.0 if controller_settings.is_integration_test else None,
                smoke_test_mode=controller_settings.is_integration_test,
            ),
        )
    except Exception as exc:
        return f"verify failed while materializing handover: {exc}"
    if not verify_result.success:
        return (
            "verify failed while materializing handover: "
            f"{verify_result.message or 'unknown verification failure'}"
        )

    try:
        submit_result = await _run_with_transient_busy_retry(
            "submit",
            lambda: client.submit(
                "script.py",
                reviewer_stage=reviewer_stage,
                episode_id=episode_id,
            ),
        )
    except Exception as exc:
        return f"submit_for_review failed while materializing handover: {exc}"
    if not submit_result.success:
        return (
            "submit_for_review failed while materializing handover: "
            f"{submit_result.message or 'unknown submit failure'}"
        )

    return None


def _plan_type_for_target(target_node: AgentName, plan_content: str) -> str:
    if "benchmark" in target_node.value or "# Learning Objective" in plan_content:
        return "benchmark"
    return "engineering"


def _seeded_schema_error(
    *,
    message: str,
    artifact_path: str | None = None,
    default_code: str = REASON_HANDOVER_INVALID,
) -> NodeEntryValidationError:
    reason = extract_benchmark_refusal_reason(message)
    return NodeEntryValidationError(
        code=reason.value if reason is not None else default_code,
        message=message,
        source=EntryValidationSource.HANDOVER,
        artifact_path=artifact_path,
    )


def _benchmark_validation_errors(
    *,
    messages: Sequence[str],
    message_prefix: str,
    artifact_path: str | None = None,
    default_code: str = REASON_HANDOVER_INVALID,
) -> list[NodeEntryValidationError]:
    refusal_errors: list[NodeEntryValidationError] = []
    fallback_errors: list[NodeEntryValidationError] = []
    for message in messages:
        prefixed_message = f"{message_prefix}{message}"
        error = _seeded_schema_error(
            message=prefixed_message,
            artifact_path=artifact_path,
            default_code=default_code,
        )
        if error.code == default_code:
            fallback_errors.append(error)
        else:
            refusal_errors.append(error)
    return refusal_errors + fallback_errors


async def validate_seeded_workspace_handoff_artifacts(
    *,
    worker_client: WorkerClient,
    target_node: AgentName,
) -> list[NodeEntryValidationError]:
    """Fail-closed schema + semantic checks for seeded/direct entry workspaces."""
    errors: list[NodeEntryValidationError] = []
    contents: dict[str, str] = {}
    benchmark_definition_model: BenchmarkDefinition | None = None
    benchmark_assembly_definition_model: AssemblyDefinition | None = None
    manufacturing_config_model = None

    if target_node in {
        AgentName.BENCHMARK_PLANNER,
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
    }:
        manufacturing_raw = await worker_client.read_file_optional(
            "manufacturing_config.yaml"
        )
        if manufacturing_raw is None:
            errors.append(
                _seeded_schema_error(
                    message=(
                        "manufacturing_config.yaml missing for planner handoff "
                        "pricing source"
                    ),
                    artifact_path="manufacturing_config.yaml",
                )
            )
        else:
            try:
                manufacturing_config_model = (
                    load_planner_manufacturing_config_from_text(manufacturing_raw)
                )
            except Exception as exc:
                errors.append(
                    _seeded_schema_error(
                        message=(
                            "manufacturing_config.yaml invalid for planner handoff "
                            f"pricing source: {exc}"
                        ),
                        artifact_path="manufacturing_config.yaml",
                    )
                )

    for rel_path in SCHEMA_BACKED_HANDOFF_PATHS:
        content = await worker_client.read_file_optional(rel_path)
        if content is not None:
            contents[rel_path] = content

    for rel_path, content in contents.items():
        if rel_path == "plan.md":
            plan_type = _plan_type_for_target(target_node, content)
            is_valid, plan_errors = validate_plan_md_structure(content, plan_type)
            if not is_valid:
                errors.extend(
                    _seeded_schema_error(
                        message=f"{rel_path}: {message}",
                        artifact_path=rel_path,
                    )
                    for message in plan_errors
                )
            continue

        if rel_path == "todo.md":
            todo_result = validate_todo_md(content)
            if not todo_result.is_valid:
                errors.extend(
                    _seeded_schema_error(
                        message=f"{rel_path}: {message}",
                        artifact_path=rel_path,
                    )
                    for message in todo_result.violations
                )
            continue

        if rel_path == "plan_refusal.md":
            is_valid, refusal_errors = validate_plan_refusal(content)
            if not is_valid and isinstance(refusal_errors, list):
                errors.extend(
                    _seeded_schema_error(
                        message=f"{rel_path}: {message}",
                        artifact_path=rel_path,
                    )
                    for message in refusal_errors
                )
            continue

        if rel_path == "benchmark_definition.yaml":
            is_valid, benchmark_result = validate_benchmark_definition_yaml(
                content,
                session_id=worker_client.session_id,
            )
            if not is_valid and isinstance(benchmark_result, list):
                errors.extend(
                    _seeded_schema_error(
                        message=f"{rel_path}: {message}",
                        artifact_path=rel_path,
                    )
                    for message in benchmark_result
                )
            elif isinstance(benchmark_result, BenchmarkDefinition):
                benchmark_definition_model = benchmark_result
            continue

        if rel_path in {
            "assembly_definition.yaml",
            "benchmark_assembly_definition.yaml",
        }:
            is_valid, assembly_result = validate_assembly_definition_yaml(
                content,
                session_id=worker_client.session_id,
                manufacturing_config=manufacturing_config_model,
            )
            if not is_valid and isinstance(assembly_result, list):
                errors.extend(
                    _seeded_schema_error(
                        message=f"{rel_path}: {message}",
                        artifact_path=rel_path,
                    )
                    for message in assembly_result
                )
            elif isinstance(assembly_result, AssemblyDefinition):
                if rel_path == "benchmark_assembly_definition.yaml":
                    benchmark_assembly_definition_model = assembly_result
                    motion_errors = validate_benchmark_assembly_motion_contract(
                        benchmark_definition=benchmark_definition_model,
                        assembly_definition=assembly_result,
                        plan_text=contents.get("plan.md"),
                        todo_text=contents.get("todo.md"),
                        plan_refusal_text=contents.get("plan_refusal.md"),
                    )
                    errors.extend(
                        _seeded_schema_error(
                            message=f"{rel_path}: {message}",
                            artifact_path=rel_path,
                        )
                        for message in motion_errors
                    )
            continue

        try:
            if rel_path == "validation_results.json":
                ValidationResultRecord.model_validate_json(content)
            elif rel_path == "simulation_result.json":
                SimulationResult.model_validate_json(content)
            elif rel_path in {
                ".manifests/benchmark_plan_review_manifest.json",
                ".manifests/engineering_plan_review_manifest.json",
            }:
                PlanReviewManifest.model_validate_json(content)
            elif rel_path in {
                ".manifests/benchmark_review_manifest.json",
                ".manifests/engineering_execution_review_manifest.json",
                ".manifests/electronics_review_manifest.json",
            }:
                ReviewManifest.model_validate_json(content)
        except Exception as exc:  # pragma: no cover - defensive parse guard
            errors.append(
                _seeded_schema_error(
                    message=f"{rel_path}: {exc}",
                    artifact_path=rel_path,
                )
            )

    present_paths = set(contents)

    if {
        "benchmark_definition.yaml",
        "assembly_definition.yaml",
    }.issubset(present_paths):
        handover_error = await validate_planner_artifacts_cross_contract(
            worker_client,
            expected_stage=AgentName.ENGINEER_PLAN_REVIEWER,
        )
        if handover_error is not None:
            errors.append(
                _seeded_schema_error(
                    message=f"engineering planner handoff: {handover_error}",
                    artifact_path="assembly_definition.yaml",
                )
            )

    if (
        benchmark_definition_model is not None
        and benchmark_assembly_definition_model is not None
    ):
        if manufacturing_config_model is None:
            manufacturing_raw = await worker_client.read_file_optional(
                "manufacturing_config.yaml"
            )
            if manufacturing_raw is not None:
                try:
                    manufacturing_config_model = (
                        load_planner_manufacturing_config_from_text(manufacturing_raw)
                    )
                except Exception as exc:
                    errors.append(
                        _seeded_schema_error(
                            message=(
                                "manufacturing_config.yaml invalid for benchmark "
                                f"handoff pricing source: {exc}"
                            ),
                            artifact_path="manufacturing_config.yaml",
                        )
                    )

        if manufacturing_config_model is not None:
            handover_errors = validate_planner_handoff_cross_contract(
                benchmark_definition=benchmark_definition_model,
                assembly_definition=benchmark_assembly_definition_model,
                manufacturing_config=manufacturing_config_model,
            )
            errors.extend(
                _seeded_schema_error(
                    message=f"benchmark_assembly_definition.yaml: {message}",
                    artifact_path="benchmark_assembly_definition.yaml",
                )
                for message in handover_errors
            )

    render_error = await validate_render_images_non_black(worker_client)
    if render_error is not None:
        errors.append(
            _seeded_schema_error(
                message=render_error,
                artifact_path="renders",
            )
        )

    return errors


class ArtifactExistsFn(Protocol):
    async def __call__(self, path: str) -> bool: ...


class CustomEntryCheck(Protocol):
    async def __call__(
        self,
        *,
        contract: NodeEntryContract,
        state: BaseModel | Mapping[str, Any],
    ) -> Sequence[NodeEntryValidationError]: ...


def integration_mode_enabled() -> bool:
    return bool(agent_settings.is_integration_test)


def get_previous_node(
    target_node: AgentName,
    *,
    graph: ValidationGraph,
) -> AgentName | None:
    return PREVIOUS_NODE_MAPS[graph].get(target_node)


def resolve_failure_disposition(
    *,
    integration_mode: bool,
    reroute_target: AgentName | None,
    integration_policy: EntryFailureDisposition,
) -> EntryFailureDisposition:
    if integration_mode:
        return integration_policy
    if reroute_target is None:
        return EntryFailureDisposition.FAIL_FAST
    return EntryFailureDisposition.REROUTE_PREVIOUS


def _get_state_value(state: BaseModel | Mapping[str, Any], field_name: str) -> Any:
    if isinstance(state, Mapping):
        return state.get(field_name)
    return getattr(state, field_name, None)


async def evaluate_node_entry_contract(
    *,
    contract: NodeEntryContract,
    state: BaseModel | Mapping[str, Any],
    artifact_exists: ArtifactExistsFn,
    graph: ValidationGraph,
    custom_checks: Mapping[str, CustomEntryCheck] | None = None,
    integration_mode: bool | None = None,
) -> NodeEntryValidationResult:
    errors: list[NodeEntryValidationError] = []

    for required_field in contract.required_state_fields:
        if _get_state_value(state, required_field) is None:
            errors.append(
                NodeEntryValidationError(
                    code=REASON_STATE_INVALID,
                    message=(
                        f"Required state field '{required_field}' is missing or null."
                    ),
                    source=EntryValidationSource.STATE,
                )
            )

    for required_artifact in contract.required_artifacts:
        if not await artifact_exists(required_artifact):
            errors.append(
                NodeEntryValidationError(
                    code=REASON_MISSING_ARTIFACT,
                    message=f"Required artifact '{required_artifact}' is missing.",
                    source=EntryValidationSource.ARTIFACT,
                    artifact_path=required_artifact,
                )
            )

    if contract.custom_check:
        custom_check = (custom_checks or {}).get(contract.custom_check)
        if custom_check is None:
            errors.append(
                NodeEntryValidationError(
                    code=REASON_POLICY_INVALID,
                    message=(
                        f"Unknown custom check '{contract.custom_check}' "
                        "configured in node entry contract."
                    ),
                    source=EntryValidationSource.POLICY,
                )
            )
        else:
            try:
                custom_errors = await custom_check(contract=contract, state=state)
                errors.extend(custom_errors)
            except Exception as exc:  # pragma: no cover - defensive guard
                errors.append(
                    NodeEntryValidationError(
                        code=REASON_CUSTOM_CHECK_FAILED,
                        message=(
                            f"Custom check '{contract.custom_check}' raised: {exc}"
                        ),
                        source=EntryValidationSource.POLICY,
                    )
                )

    if not errors:
        return NodeEntryValidationResult(
            ok=True,
            target_node=contract.node,
            disposition=EntryFailureDisposition.ALLOW,
            errors=[],
            reroute_target=None,
            reason_code=REASON_OK,
        )

    reroute_target = get_previous_node(contract.node, graph=graph)
    resolved_integration_mode = (
        integration_mode_enabled() if integration_mode is None else integration_mode
    )
    disposition = resolve_failure_disposition(
        integration_mode=resolved_integration_mode,
        reroute_target=reroute_target,
        integration_policy=contract.integration_failure_policy,
    )

    if (
        disposition == EntryFailureDisposition.FAIL_FAST
        and reroute_target is None
        and not resolved_integration_mode
    ):
        errors.append(
            NodeEntryValidationError(
                code=REASON_NO_PREVIOUS_NODE,
                message=(
                    f"No deterministic previous node mapping exists for "
                    f"'{contract.node.value}'."
                ),
                source=EntryValidationSource.POLICY,
            )
        )

    return NodeEntryValidationResult(
        ok=False,
        target_node=contract.node,
        disposition=disposition,
        errors=errors,
        reroute_target=reroute_target,
        reason_code=errors[0].code,
    )


__all__ = [
    "BENCHMARK_CODER_HANDOVER_CHECK",
    "BENCHMARK_PLANNER_HANDOFF_ARTIFACTS",
    "BENCHMARK_PLAN_REVIEWER_HANDOVER_CHECK",
    "BENCHMARK_PLAN_REVIEW_MANIFEST",
    "BENCHMARK_PREVIOUS_NODE_MAP",
    "BENCHMARK_REVIEWER_HANDOVER_CHECK",
    "ENGINEER_BENCHMARK_CONTEXT_ARTIFACTS",
    "ENGINEER_BENCHMARK_HANDOVER_CHECK",
    "ENGINEER_EXECUTION_REVIEWER_HANDOVER_CHECK",
    "ENGINEER_PLANNER_HANDOFF_ARTIFACTS",
    "ENGINEER_PREVIOUS_NODE_MAP",
    "PREVIOUS_NODE_MAPS",
    "REASON_CUSTOM_CHECK_FAILED",
    "REASON_HANDOVER_INVALID",
    "REASON_MISSING_ARTIFACT",
    "REASON_NO_PREVIOUS_NODE",
    "REASON_OK",
    "REASON_POLICY_INVALID",
    "REASON_REVIEWER_ENTRY_BLOCKED",
    "REASON_STATE_INVALID",
    "REVIEWER_HANDOFF_ARTIFACTS",
    "NodeEntryContract",
    "NodeEntryValidationError",
    "NodeEntryValidationResult",
    "ValidationGraph",
    "benchmark_coder_handover_custom_check",
    "benchmark_coder_handover_custom_check_from_session_id",
    "benchmark_plan_reviewer_handover_custom_check",
    "benchmark_plan_reviewer_handover_custom_check_from_session_id",
    "build_benchmark_node_contracts",
    "build_engineer_node_contracts",
    "engineer_benchmark_handover_custom_check",
    "evaluate_node_entry_contract",
    "get_previous_node",
    "integration_mode_enabled",
    "resolve_failure_disposition",
    "reviewer_handover_custom_check_from_session_id",
    "validate_seeded_workspace_handoff_artifacts",
]
