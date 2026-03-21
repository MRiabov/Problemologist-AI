import asyncio
import os
import uuid

import httpx
import pytest

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkConfirmResponse,
    EpisodeCreateResponse,
    EpisodeResponse,
)
from shared.enums import AgentName, EntryFailureDisposition, EpisodeStatus, TraceType
from shared.models.schemas import EntryValidationContext

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


async def _poll_engineer_episode(
    client: httpx.AsyncClient,
    episode_id: str,
    *,
    terminal_statuses: set[EpisodeStatus],
    max_attempts: int = 90,
) -> EpisodeResponse:
    latest: EpisodeResponse | None = None
    for _ in range(max_attempts):
        resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert resp.status_code == 200, resp.text
        latest = EpisodeResponse.model_validate(resp.json())
        if latest.status in terminal_statuses:
            return latest
        await asyncio.sleep(1)
    assert latest is not None
    return latest


async def _poll_benchmark_session(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    terminal_statuses: set[EpisodeStatus],
    max_attempts: int = 90,
) -> EpisodeResponse:
    latest: EpisodeResponse | None = None
    for _ in range(max_attempts):
        resp = await client.get(f"{CONTROLLER_URL}/api/benchmark/{session_id}")
        if resp.status_code == 404:
            await asyncio.sleep(1)
            continue
        assert resp.status_code == 200, resp.text
        latest = EpisodeResponse.model_validate(resp.json())
        if latest.status in terminal_statuses:
            return latest
        await asyncio.sleep(1)
    assert latest is not None
    return latest


def _entry_validation_from_episode(episode: EpisodeResponse) -> EntryValidationContext:
    assert episode.metadata_vars is not None
    additional_info = episode.metadata_vars.additional_info or {}
    raw = additional_info.get("entry_validation")
    assert isinstance(raw, dict), "Missing metadata.additional_info.entry_validation"
    return EntryValidationContext.model_validate(raw)


def _node_start_traces(episode: EpisodeResponse, node_name: str) -> list[str]:
    return [
        trace.content or ""
        for trace in (episode.traces or [])
        if trace.trace_type == TraceType.LOG
        and trace.name == node_name
        and "Starting task phase" in (trace.content or "")
    ]


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "node_entry_validation_rejected",
        "execution reviewer entry blocked",
        "reviewer handover",
    ]
)
@pytest.mark.asyncio
async def test_int_184_engineer_fail_fast_and_skip_target_node():
    """
    INT-184: Engineer path entry validation must fail-fast, skip target node execution,
    and persist structured metadata including reroute_target for non-integration parity.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-184 engineer node-entry fail-fast contract.",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_EXECUTION_REVIEWER,
            start_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _poll_engineer_episode(
            client,
            str(run.episode_id),
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.COMPLETED},
        )
        assert episode.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(episode)
        assert entry.node == AgentName.ENGINEER_EXECUTION_REVIEWER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "reviewer_entry_blocked"
        assert entry.reroute_target == AgentName.ENGINEER_CODER
        assert entry.errors
        assert all(err.code and err.message and err.source for err in entry.errors)

        additional_info = episode.metadata_vars.additional_info or {}
        assert additional_info.get("entry_validation_terminal") is True
        assert not _node_start_traces(
            episode, AgentName.ENGINEER_EXECUTION_REVIEWER.value
        )

        failure_logs = [
            log
            for log in (episode.metadata_vars.validation_logs or [])
            if log.startswith("ENTRY_VALIDATION_FAILED[")
        ]
        assert len(failure_logs) == 1

        failure_traces = [
            t
            for t in (episode.traces or [])
            if t.name == "node_entry_validation_failed"
            and t.trace_type in {TraceType.EVENT, TraceType.ERROR}
        ]
        assert failure_traces, "Expected node_entry_validation_failed traces."


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "node_entry_validation_rejected",
        "benchmark_assembly_definition.yaml",
        "missing_artifact",
    ]
)
@pytest.mark.asyncio
async def test_int_184_engineer_planner_requires_benchmark_assembly_definition():
    """
    INT-184: Engineer planner entry must fail closed when benchmark_assembly_definition.yaml is absent.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-184-{uuid.uuid4().hex[:8]}"
        req = AgentRunRequest(
            task="INT-184 engineer planner requires benchmark assembly context.",
            session_id=session_id,
            agent_name=AgentName.ENGINEER_PLANNER,
            start_node=AgentName.ENGINEER_PLANNER,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202, run_resp.text
        run = AgentRunResponse.model_validate(run_resp.json())

        episode = await _poll_engineer_episode(
            client,
            str(run.episode_id),
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.COMPLETED},
        )
        assert episode.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(episode)
        assert entry.node == AgentName.ENGINEER_PLANNER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "missing_artifact"
        assert entry.reroute_target is None
        assert any(
            error.artifact_path == "benchmark_assembly_definition.yaml"
            for error in entry.errors
        )
        assert any(
            "benchmark_assembly_definition.yaml" in error.message
            for error in entry.errors
        )
        assert not _node_start_traces(episode, AgentName.ENGINEER_PLANNER.value)


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "continue_generation_invalid_episode_status",
    ]
)
@pytest.mark.asyncio
async def test_int_184_benchmark_fail_fast_state_invalid_confirm_path():
    """
    INT-184: Benchmark path invalid entry must fail-fast with structured metadata
    and no benchmark_coder node execution evidence.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        create_req = AgentRunRequest(
            task="INT-184 benchmark state-invalid confirm path.",
            session_id=f"INT-184-benchmark-{uuid.uuid4().hex[:8]}",
        )
        create_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json=create_req.model_dump(mode="json"),
        )
        assert create_resp.status_code == 201, create_resp.text
        created = EpisodeCreateResponse.model_validate(create_resp.json())
        session_id = str(created.episode_id)

        confirm_resp = await client.post(
            f"{CONTROLLER_URL}/api/benchmark/{session_id}/confirm",
            json={"comment": "INT-184 invalid status", "additional_turns": 0},
        )
        assert confirm_resp.status_code == 200, confirm_resp.text
        BenchmarkConfirmResponse.model_validate(confirm_resp.json())

        session = await _poll_benchmark_session(
            client,
            session_id,
            terminal_statuses={EpisodeStatus.FAILED, EpisodeStatus.PLANNED},
        )
        assert session.status == EpisodeStatus.FAILED

        entry = _entry_validation_from_episode(session)
        assert entry.node == AgentName.BENCHMARK_CODER
        assert entry.disposition == EntryFailureDisposition.FAIL_FAST
        assert entry.reason_code == "state_invalid"
        assert entry.reroute_target is None
        assert entry.errors

        assert not _node_start_traces(session, AgentName.BENCHMARK_CODER.value)
        assert any(
            "continue_generation_session requires PLANNED episode status" in log
            for log in (session.metadata_vars.validation_logs or [])
        )
