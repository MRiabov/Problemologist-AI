import asyncio
import json
import os
import uuid

import httpx
import pytest

from shared.enums import AgentName
from shared.workers.schema import ReadFileRequest, WriteFileRequest
from tests.integration.agent.helpers import (
    CONTROLLER_URL,
    get_controller_log_path,
    read_log_segment,
    run_agent_episode,
    strip_ansi,
    wait_for_episode_terminal,
    wait_for_queue_empty,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")


def _tool_trace_indexes(
    traces: list[dict],
    *,
    name: str,
    content_substring: str | None = None,
) -> list[int]:
    indexes: list[int] = []
    for idx, trace in enumerate(traces):
        if trace.get("trace_type") != "TOOL_START":
            continue
        if trace.get("name") != name:
            continue
        content = trace.get("content") or ""
        if content_substring and content_substring not in content:
            continue
        indexes.append(idx)
    return indexes


def _trace_blob(episode_payload: dict) -> str:
    traces = episode_payload.get("traces", [])
    return json.dumps(traces, sort_keys=True)


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_181_tool_loop_ordering_and_clean_termination():
    """INT-181: Tool-call ordering is preserved and run terminates cleanly."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        _, episode_id = await run_agent_episode(
            client,
            int_id="INT-181",
            task="INT-181 ordering contract",
            agent_name=AgentName.ENGINEER_CODER,
        )
        episode = await wait_for_episode_terminal(client, episode_id)

    assert episode["status"] != "FAILED", f"Episode failed: {episode}"

    traces = episode.get("traces", [])
    list_files_idxs = _tool_trace_indexes(traces, name="list_files")
    order_probe_write_idxs = _tool_trace_indexes(
        traces,
        name="write_file",
        content_substring="journal.md",
    )

    assert list_files_idxs, "Expected at least one list_files TOOL_START trace."
    assert len(order_probe_write_idxs) == 1, (
        "Expected exactly one write_file TOOL_START trace for journal.md."
    )
    assert list_files_idxs[0] < order_probe_write_idxs[0], (
        "Expected list_files to occur before write_file(journal.md)."
    )
    assert any(
        (trace.get("content") or "").startswith("Agent finished execution")
        for trace in traces
        if trace.get("trace_type") == "LOG"
    ), "Expected final completion log trace in episode trace stream."


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_182_concurrent_agent_run_isolation_files_traces_context():
    """INT-182: Concurrent runs remain isolated across filesystem and observability."""
    token_a = f"INT182-A-{uuid.uuid4().hex}"
    token_b = f"INT182-B-{uuid.uuid4().hex}"
    session_a = f"INT-182-{uuid.uuid4().hex[:8]}"
    session_b = f"INT-182-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=300.0) as client:
        write_a = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(path="secret_a.txt", content=token_a).model_dump(),
            headers={"X-Session-ID": session_a},
        )
        assert write_a.status_code == 200, write_a.text

        write_b = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(path="secret_b.txt", content=token_b).model_dump(),
            headers={"X-Session-ID": session_b},
        )
        assert write_b.status_code == 200, write_b.text

        run_a, run_b = await asyncio.gather(
            client.post(
                f"{CONTROLLER_URL}/api/agent/run",
                json={
                    "task": f"INT-182 concurrent run A token: {token_a}",
                    "session_id": session_a,
                    "agent_name": AgentName.ENGINEER_CODER,
                },
            ),
            client.post(
                f"{CONTROLLER_URL}/api/agent/run",
                json={
                    "task": f"INT-182 concurrent run B token: {token_b}",
                    "session_id": session_b,
                    "agent_name": AgentName.ENGINEER_CODER,
                },
            ),
        )
        assert run_a.status_code == 202, run_a.text
        assert run_b.status_code == 202, run_b.text

        episode_a_id = run_a.json()["episode_id"]
        episode_b_id = run_b.json()["episode_id"]

        episode_a, episode_b = await asyncio.gather(
            wait_for_episode_terminal(client, episode_a_id),
            wait_for_episode_terminal(client, episode_b_id),
        )

        assert episode_a["status"] != "FAILED", f"Episode A failed: {episode_a}"
        assert episode_b["status"] != "FAILED", f"Episode B failed: {episode_b}"

        read_a_from_b = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="secret_a.txt").model_dump(),
            headers={"X-Session-ID": session_b},
        )
        read_b_from_a = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="secret_b.txt").model_dump(),
            headers={"X-Session-ID": session_a},
        )
        assert read_a_from_b.status_code == 404, read_a_from_b.text
        assert read_b_from_a.status_code == 404, read_b_from_a.text

    blob_a = _trace_blob(episode_a)
    blob_b = _trace_blob(episode_b)

    assert token_a in blob_a, "Run A token missing from run A traces."
    assert token_b in blob_b, "Run B token missing from run B traces."
    assert token_a not in blob_b, "Run A token leaked into run B traces."
    assert token_b not in blob_a, "Run B token leaked into run A traces."


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_183_steerability_queue_single_consumption():
    """INT-183: Steer prompt is queued once, consumed once, then queue stays empty."""
    steer_text = f"INT-183 steer text {uuid.uuid4().hex}"
    log_path = get_controller_log_path()
    start_offset = log_path.stat().st_size if log_path.exists() else 0

    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-183",
            task="INT-183 queue consumption contract",
            agent_name=AgentName.ENGINEER_CODER,
        )

        steer_resp = await client.post(
            f"{CONTROLLER_URL}/api/v1/sessions/{session_id}/steer",
            json={
                "text": steer_text,
                "selections": [],
                "mentions": [],
                "code_references": [],
            },
        )
        assert steer_resp.status_code == 202, steer_resp.text
        assert steer_resp.json().get("status") == "queued", steer_resp.json()

        await wait_for_queue_empty(client, session_id, timeout_s=90.0)
        episode = await wait_for_episode_terminal(client, episode_id)
        assert episode["status"] != "FAILED", f"Episode failed: {episode}"

        queue_after = await client.get(
            f"{CONTROLLER_URL}/api/v1/sessions/{session_id}/queue"
        )
        assert queue_after.status_code == 200, queue_after.text
        assert queue_after.json() == [], "Queue should remain empty after consumption."

    log_segment = strip_ansi(read_log_segment(log_path, start_offset))
    assert "steerability_interruption" in log_segment, (
        "Expected steerability_interruption log for queued prompt consumption."
    )
    assert f"session_id={session_id}" in log_segment, (
        "Expected consumed steering log to include the run session_id."
    )
    assert steer_text in log_segment, "Expected steer text in consumption log entry."
