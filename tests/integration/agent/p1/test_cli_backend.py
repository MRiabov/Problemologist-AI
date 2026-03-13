import os
from pathlib import Path

import httpx
import pytest

from shared.enums import AgentName
from tests.integration.agent.helpers import (
    run_agent_episode,
    wait_for_episode_terminal,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
CONTROLLER_ERROR_LOG = Path("logs/integration_tests/json/controller_errors.json")

CLI_BACKEND_ENABLED = os.getenv("INTEGRATION_USE_REAL_LLM", "").lower() in {
    "1",
    "true",
    "yes",
}


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
@pytest.mark.skipif(
    not CLI_BACKEND_ENABLED,
    reason="Requires INTEGRATION_USE_REAL_LLM=1.",
)
async def test_int_189_cli_backend_executes_planner_via_worker_runtime():
    """
    INT-189: CLI-backed planner execution runs through worker runtime and
    submit_plan gate.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-189",
            task="INT-189 CLI planner smoke",
            agent_name=AgentName.ENGINEER_PLANNER,
        )
        episode = await wait_for_episode_terminal(client, episode_id)

        assert episode["status"] in {"PLANNED", "COMPLETED"}, episode

        traces = episode.get("traces", [])
        submit_plan_traces = [
            trace
            for trace in traces
            if trace.get("trace_type") == "TOOL_START"
            and trace.get("name") == "submit_plan"
        ]
        assert submit_plan_traces, "Expected submit_plan TOOL_START trace."
        assert any(
            '"status": "submitted"'
            in str((trace.get("metadata_vars") or {}).get("observation", ""))
            or "'status': 'submitted'"
            in str((trace.get("metadata_vars") or {}).get("observation", ""))
            for trace in submit_plan_traces
        ), submit_plan_traces

        plan_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "plan.md"},
            headers={"X-Session-ID": session_id},
        )
        assert plan_resp.status_code == 200, plan_resp.text
        assert "# Engineering Plan" in plan_resp.json()["content"]

        assembly_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "assembly_definition.yaml"},
            headers={"X-Session-ID": session_id},
        )
        assert assembly_resp.status_code == 200, assembly_resp.text
        assert "planner_target_max_unit_cost_usd" in assembly_resp.json()["content"]


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
@pytest.mark.skipif(
    not CLI_BACKEND_ENABLED,
    reason="Requires INTEGRATION_USE_REAL_LLM=1.",
)
@pytest.mark.allow_backend_errors("engineer_planner_cli_bootstrap_failed")
async def test_int_190_cli_backend_auth_prompt_fails_without_retry():
    """
    INT-190: interactive CLI bootstrap/auth prompts fail closed without burning
    CLI retries.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-190",
            task="INT-190 CLI auth prompt fail closed",
            agent_name=AgentName.ENGINEER_PLANNER,
        )
        episode = await wait_for_episode_terminal(client, episode_id, timeout_s=60.0)

        assert episode["status"] == "FAILED", episode

        traces = episode.get("traces", [])
        assert not any(
            trace.get("trace_type") == "TOOL_START"
            and trace.get("name") == "submit_plan"
            for trace in traces
        ), traces

        assert CONTROLLER_ERROR_LOG.exists(), CONTROLLER_ERROR_LOG
        matching_errors = [
            line
            for line in CONTROLLER_ERROR_LOG.read_text(encoding="utf-8").splitlines()
            if session_id in line and "engineer_planner_cli_bootstrap_failed" in line
        ]
        assert len(matching_errors) == 1, matching_errors
