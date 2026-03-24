import asyncio
import os
import uuid

import httpx
import pytest
import yaml

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import AgentName, EpisodeStatus, ReviewDecision
from tests.integration.agent.helpers import (
    seed_benchmark_assembly_definition,
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


async def _run_and_wait(
    client: httpx.AsyncClient, *, agent_name: AgentName, session_id: str, task: str
) -> EpisodeResponse:
    await seed_benchmark_assembly_definition(
        client,
        session_id,
        benchmark_max_unit_cost_usd=250.0,
        benchmark_max_weight_g=2500.0,
        planner_target_max_unit_cost_usd=250.0,
        planner_target_max_weight_g=2500.0,
    )
    req = AgentRunRequest(task=task, session_id=session_id, agent_name=agent_name)
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
        # Plan reviewer stage gate
        plan_session = f"INT-074-{uuid.uuid4().hex[:8]}"
        plan_episode = await _run_and_wait(
            client,
            agent_name=AgentName.ENGINEER_CODER,
            session_id=plan_session,
            task="INT-074 plan reviewer DOF minimization gate",
        )
        plan_dof_events = [
            t
            for t in plan_episode.traces
            if t.name == "excessive_dof_detected"
            and "engineering_plan_reviewer" in str(t.content or "")
        ]
        assert plan_dof_events, (
            "Expected excessive_dof_detected event from plan reviewer stage"
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
            int_id="INT-075",
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
