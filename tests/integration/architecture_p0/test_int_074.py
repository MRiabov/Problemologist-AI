import ast
import asyncio
import os
import uuid
from pathlib import Path

import httpx
import pytest
import yaml

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import AgentName, EpisodeStatus, ReviewDecision
from tests.integration.agent.helpers import (
    seed_benchmark_assembly_definition,
    seed_current_revision_render_preview,
    seed_execution_reviewer_handover,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


def _has_expected_review_evidence(
    episode: EpisodeResponse, *, required_checklist_pairs: tuple[tuple[str, str], ...]
) -> bool:
    traces = episode.traces or []
    review_traces = [
        trace
        for trace in traces
        if trace.name == "review_decision" and trace.metadata_vars is not None
    ]
    return all(
        any(
            trace.metadata_vars.checklist.get(checklist_key) == expected_value
            for trace in review_traces
        )
        for checklist_key, expected_value in required_checklist_pairs
    )


def _parse_event_payload(trace) -> dict:
    if not trace.content:
        return {}
    try:
        payload = ast.literal_eval(trace.content)
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _is_inspect_media_trace(trace) -> bool:
    return (
        trace.name in {"inspect_media", "inspect_media_tool"}
        and getattr(trace.trace_type, "value", str(trace.trace_type)) == "TOOL_START"
    )


def _expected_minimal_dofs_payload(trace) -> dict[str, object]:
    payload = _parse_event_payload(trace)
    assert payload, f"Expected event payload for trace {trace}"
    assert payload.get("dof_count_gt_3") is True, payload
    assert isinstance(payload.get("expected_minimal_engineering_dofs"), list), payload
    assert isinstance(payload.get("expected_minimal_dofs"), list), payload
    assert (
        payload["expected_minimal_engineering_dofs"] == payload["expected_minimal_dofs"]
    ), payload
    assert len(payload["expected_minimal_engineering_dofs"]) == 3, payload
    assert len(payload["expected_minimal_dofs"]) == 3, payload
    return payload


async def _wait_for_trace(
    episode_id: str,
    *,
    trace_name: str,
    predicate,
    attempts: int = 10,
) -> list:
    episode: EpisodeResponse | None = None
    for _ in range(attempts):
        await asyncio.sleep(1.0)
        async with httpx.AsyncClient(timeout=30.0) as client:
            ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
            assert ep_resp.status_code == 200, ep_resp.text
            episode = EpisodeResponse.model_validate(ep_resp.json())
        traces = [
            trace
            for trace in (episode.traces or [])
            if trace.name == trace_name and predicate(trace)
        ]
        if traces:
            return traces
    assert episode is not None
    return [
        trace
        for trace in (episode.traces or [])
        if trace.name == trace_name and predicate(trace)
    ]


async def _run_and_wait(
    client: httpx.AsyncClient,
    *,
    agent_name: AgentName,
    session_id: str,
    task: str,
    start_node: AgentName | None = None,
) -> EpisodeResponse:
    await seed_benchmark_assembly_definition(
        client,
        session_id,
        benchmark_max_unit_cost_usd=250.0,
        benchmark_max_weight_g=2500.0,
        planner_target_max_unit_cost_usd=250.0,
        planner_target_max_weight_g=2500.0,
    )
    req = AgentRunRequest(
        task=task,
        session_id=session_id,
        agent_name=agent_name,
        start_node=start_node,
    )
    run_resp = await client.post(
        f"{CONTROLLER_URL}/api/agent/run",
        json=req.model_dump(mode="json"),
    )
    assert run_resp.status_code == 202, run_resp.text
    run = AgentRunResponse.model_validate(run_resp.json())

    for _ in range(180):
        await asyncio.sleep(1.0)
        ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{run.episode_id}")
        assert ep_resp.status_code == 200, ep_resp.text
        episode = EpisodeResponse.model_validate(ep_resp.json())
        if episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
            EpisodeStatus.PLANNED,
        } or _has_expected_review_evidence(
            episode,
            required_checklist_pairs=(("dof_minimality", "fail"),),
        ):
            return episode

    pytest.fail(
        f"Episode {run.episode_id} did not surface the expected review evidence in time"
    )


async def _seed_plan_reviewer_handoff(
    client: httpx.AsyncClient, session_id: str
) -> None:
    for relative_path in (
        "plan.md",
        "todo.md",
        "assembly_definition.yaml",
        "benchmark_definition.yaml",
    ):
        content = Path(
            "tests/integration/mock_responses/INT-074/engineer_planner/entry_01/"
            + {
                "plan.md": "01__plan.md",
                "todo.md": "02__todo.md",
                "assembly_definition.yaml": "03__assembly_definition.yaml",
                "benchmark_definition.yaml": "04__benchmark_definition.yaml",
            }[relative_path]
        ).read_text(encoding="utf-8")
        await client.post(
            "http://127.0.0.1:18001/fs/write",
            json={
                "path": relative_path,
                "content": content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )


async def _wait_for_review_evidence(
    client: httpx.AsyncClient,
    *,
    episode_id: str,
    required_checklist_pairs: tuple[tuple[str, str], ...],
    attempts: int = 10,
) -> EpisodeResponse:
    episode: EpisodeResponse | None = None
    for _ in range(attempts):
        ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert ep_resp.status_code == 200, ep_resp.text
        episode = EpisodeResponse.model_validate(ep_resp.json())
        if _has_expected_review_evidence(
            episode,
            required_checklist_pairs=required_checklist_pairs,
        ):
            return episode
        await asyncio.sleep(1.0)

    assert episode is not None
    return episode


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_074_engineering_dof_minimization_review_gate():
    """
    INT-074: Plan/execution reviewers must flag excessive DOFs (`len(dofs) > 3`)
    unless explicit accepted justification is present.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        plan_session = f"INT-074-{uuid.uuid4().hex[:8]}"
        await seed_benchmark_assembly_definition(
            client,
            plan_session,
            benchmark_max_unit_cost_usd=250.0,
            benchmark_max_weight_g=2500.0,
            planner_target_max_unit_cost_usd=250.0,
            planner_target_max_weight_g=2500.0,
        )
        await _seed_plan_reviewer_handoff(client, plan_session)
        await seed_current_revision_render_preview(client, session_id=plan_session)
        planner_episode = await _run_and_wait(
            client,
            agent_name=AgentName.ENGINEER_PLANNER,
            session_id=plan_session,
            task="INT-074 plan reviewer DOF minimization gate",
        )
        assert planner_episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
            EpisodeStatus.PLANNED,
        }, planner_episode

        await seed_current_revision_render_preview(client, session_id=plan_session)
        plan_episode = await _run_and_wait(
            client,
            agent_name=AgentName.ENGINEER_PLAN_REVIEWER,
            session_id=plan_session,
            task="INT-074 plan reviewer DOF minimization gate",
            start_node=AgentName.ENGINEER_PLAN_REVIEWER,
        )
        assert _has_expected_review_evidence(
            plan_episode,
            required_checklist_pairs=(("dof_minimality", "fail"),),
        ), plan_episode
        plan_review_traces = [
            trace
            for trace in plan_episode.traces
            if trace.name == "review_decision" and trace.metadata_vars is not None
        ]
        assert plan_review_traces, "Expected plan review_decision trace"
        latest_plan_review_trace = max(plan_review_traces, key=lambda trace: trace.id)
        inspect_media_traces = [
            trace for trace in plan_episode.traces if _is_inspect_media_trace(trace)
        ]
        assert inspect_media_traces, (
            "Plan reviewer rejection must call inspect_media()."
        )
        assert any(
            trace.id < latest_plan_review_trace.id for trace in inspect_media_traces
        ), "inspect_media must occur before the final plan review decision."
        plan_dof_event_traces = [
            trace
            for trace in plan_episode.traces
            if trace.name == "excessive_dof_detected"
        ]
        assert plan_dof_event_traces, "Expected excessive_dof_detected trace"
        plan_dof_event = next(
            trace
            for trace in plan_dof_event_traces
            if _parse_event_payload(trace).get("reviewer_stage")
            == "engineering_plan_reviewer"
        )
        plan_dof_payload = _expected_minimal_dofs_payload(plan_dof_event)
        assert plan_dof_payload["part_id"] == "over_actuated_link", plan_dof_payload
        assert plan_dof_payload["expected_minimal_engineering_dofs"] == [
            "tx",
            "ty",
            "tz",
        ], plan_dof_payload
        assert any(trace.id < plan_dof_event.id for trace in inspect_media_traces), (
            "inspect_media must occur before the excessive_dof_detected decision."
        )
        assert plan_episode.status != EpisodeStatus.COMPLETED
        plan_decision_path = "reviews/engineering-plan-review-decision-round-1.yaml"
        plan_comments_path = "reviews/engineering-plan-review-comments-round-1.yaml"
        plan_comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{plan_episode.id}/assets/{plan_comments_path}"
        )
        assert plan_comments_resp.status_code == 200, plan_comments_resp.text
        plan_comments = yaml.safe_load(plan_comments_resp.text)
        assert plan_comments["summary"].startswith("REJECTED:"), plan_comments
        assert plan_comments["checklist"]["dof_minimality"] == "fail", plan_comments
        plan_decision_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{plan_episode.id}/assets/{plan_decision_path}"
        )
        assert plan_decision_resp.status_code == 200, plan_decision_resp.text

        # Justified-motion path: the approved plan must persist the canonical
        # DOF checklist key.
        justified_session = f"INT-075-{uuid.uuid4().hex[:8]}"
        await seed_execution_reviewer_handover(
            client,
            session_id=justified_session,
            int_id="INT-181",
        )
        justified_episode = await _run_and_wait(
            client,
            agent_name=AgentName.ENGINEER_CODER,
            session_id=justified_session,
            task="INT-074 justified-motion acceptance gate",
        )
        justified_episode = await _wait_for_review_evidence(
            client,
            episode_id=str(justified_episode.id),
            required_checklist_pairs=(
                ("dof_minimality", "pass"),
                ("dof_deviation_justified", "pass"),
            ),
        )
        assert _has_expected_review_evidence(
            justified_episode,
            required_checklist_pairs=(
                ("dof_minimality", "pass"),
                ("dof_deviation_justified", "pass"),
            ),
        ), justified_episode
        justified_review_traces = [
            t
            for t in justified_episode.traces
            if t.name == "review_decision"
            and t.metadata_vars is not None
            and t.metadata_vars.decision == ReviewDecision.APPROVED
        ]
        assert justified_review_traces, (
            "Expected approved review_decision trace for justified DOF path"
        )
        assert any(
            trace.metadata_vars.checklist.get("dof_minimality") == "pass"
            for trace in justified_review_traces
        ), "Expected canonical dof_minimality checklist key in approved plan trace"

        justified_plan_comments_path = (
            "reviews/engineering-plan-review-comments-round-1.yaml"
        )
        comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{justified_episode.id}/assets/"
            f"{justified_plan_comments_path}"
        )
        assert comments_resp.status_code == 200, comments_resp.text
        comments = yaml.safe_load(comments_resp.text)
        assert comments["summary"].startswith("APPROVED:"), comments
        assert comments["checklist"]["dof_minimality"] == "pass", comments
        execution_comments_path = (
            "reviews/engineering-execution-review-comments-round-1.yaml"
        )
        execution_comments_resp = await client.get(
            f"{CONTROLLER_URL}/episodes/{justified_episode.id}/assets/"
            f"{execution_comments_path}"
        )
        assert execution_comments_resp.status_code == 200, execution_comments_resp.text
        execution_comments = yaml.safe_load(execution_comments_resp.text)
        assert execution_comments["checklist"]["dof_deviation_justified"] == "pass", (
            execution_comments
        )
