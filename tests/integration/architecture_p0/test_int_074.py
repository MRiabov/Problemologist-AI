import asyncio
import os
import uuid

import httpx
import pytest

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import AgentName, EpisodeStatus
from tests.integration.agent.helpers import seed_benchmark_assembly_definition

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


async def _run_and_wait(
    client: httpx.AsyncClient, *, agent_name: AgentName, session_id: str, task: str
) -> EpisodeResponse:
    await seed_benchmark_assembly_definition(client, session_id)
    req = AgentRunRequest(task=task, session_id=session_id, agent_name=agent_name)
    run_resp = await client.post(
        f"{CONTROLLER_URL}/api/agent/run",
        json=req.model_dump(mode="json"),
    )
    assert run_resp.status_code == 202, run_resp.text
    run = AgentRunResponse.model_validate(run_resp.json())

    for _ in range(90):
        await asyncio.sleep(1.0)
        ep_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{run.episode_id}")
        assert ep_resp.status_code == 200, ep_resp.text
        episode = EpisodeResponse.model_validate(ep_resp.json())
        if episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
            EpisodeStatus.PLANNED,
        }:
            return episode

    pytest.fail(f"Episode {run.episode_id} did not reach terminal state in time")


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

        # Execution reviewer stage gate (must still flag over-actuation)
        exec_session = f"INT-075-{uuid.uuid4().hex[:8]}"
        exec_episode = await _run_and_wait(
            client,
            agent_name=AgentName.ENGINEER_CODER,
            session_id=exec_session,
            task="INT-074 execution reviewer over-actuation gate",
        )
        exec_dof_events = [
            t
            for t in exec_episode.traces
            if t.name == "excessive_dof_detected"
            and "engineering_execution_reviewer" in str(t.content or "")
        ]
        assert exec_dof_events, (
            "Expected excessive_dof_detected event from execution reviewer stage"
        )
