from __future__ import annotations

from collections.abc import Mapping, Sequence
from enum import StrEnum
from typing import Any, Protocol

from pydantic import BaseModel, Field, model_validator

from controller.agent.config import settings as agent_settings
from shared.enums import AgentName, EntryFailureDisposition, EntryValidationSource

REASON_OK = "ok"
REASON_STATE_INVALID = "state_invalid"
REASON_MISSING_ARTIFACT = "missing_artifact"
REASON_HANDOVER_INVALID = "handover_invalid"
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
    AgentName.ENGINEER_REVIEWER: AgentName.ELECTRONICS_PLANNER,
    AgentName.ENGINEER_CODER: AgentName.ENGINEER_REVIEWER,
    AgentName.ELECTRONICS_ENGINEER: AgentName.ENGINEER_CODER,
    AgentName.ELECTRONICS_REVIEWER: AgentName.ELECTRONICS_ENGINEER,
    AgentName.EXECUTION_REVIEWER: AgentName.ELECTRONICS_REVIEWER,
    AgentName.COTS_SEARCH: AgentName.ENGINEER_PLANNER,
    AgentName.SKILL_AGENT: AgentName.EXECUTION_REVIEWER,
    AgentName.JOURNALLING_AGENT: AgentName.ENGINEER_REVIEWER,
    AgentName.STEER: None,
}

BENCHMARK_PREVIOUS_NODE_MAP: Mapping[AgentName, AgentName | None] = {
    AgentName.BENCHMARK_PLANNER: None,
    AgentName.BENCHMARK_CODER: AgentName.BENCHMARK_PLANNER,
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
    "BENCHMARK_PREVIOUS_NODE_MAP",
    "ENGINEER_PREVIOUS_NODE_MAP",
    "PREVIOUS_NODE_MAPS",
    "REASON_CUSTOM_CHECK_FAILED",
    "REASON_HANDOVER_INVALID",
    "REASON_MISSING_ARTIFACT",
    "REASON_NO_PREVIOUS_NODE",
    "REASON_OK",
    "REASON_POLICY_INVALID",
    "REASON_STATE_INVALID",
    "NodeEntryContract",
    "NodeEntryValidationError",
    "NodeEntryValidationResult",
    "ValidationGraph",
    "evaluate_node_entry_contract",
    "get_previous_node",
    "integration_mode_enabled",
    "resolve_failure_disposition",
]
