import asyncio
import json
import os
import signal
import subprocess
import uuid
from pathlib import Path

import httpx
import pytest

from shared.enums import AgentName
from shared.workers.schema import ReadFileRequest, WriteFileRequest
from tests.integration.agent.helpers import (
    CONTROLLER_URL,
    get_controller_log_path,
    read_log_segment,
    run_agent_episode,
    seed_benchmark_assembly_definition,
    seed_execution_reviewer_handover,
    strip_ansi,
    wait_for_episode_terminal,
    wait_for_queue_empty,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")


async def _wait_for_worker_light_health(timeout_s: float = 30.0) -> None:
    deadline = asyncio.get_event_loop().time() + timeout_s
    async with httpx.AsyncClient(timeout=5.0) as client:
        while asyncio.get_event_loop().time() < deadline:
            try:
                resp = await client.get(f"{WORKER_LIGHT_URL}/health")
                if resp.status_code == 200:
                    return
            except Exception:
                pass
            await asyncio.sleep(0.5)

    raise AssertionError("Worker light did not become healthy after restart.")


async def _restart_worker_light_after_outage() -> None:
    env = os.environ.copy()
    env.update(
        {
            "WORKER_TYPE": "light",
            "EXTRA_DEBUG_LOG": str(
                Path("logs/integration_tests/worker_light_debug.log")
            ),
            "EXTRA_ERROR_LOG": str(
                Path("logs/integration_tests/worker_light_errors.log")
            ),
            "EXTRA_ERROR_JSON_LOG": str(
                Path("logs/integration_tests/json/worker_light_errors.json")
            ),
        }
    )

    log_path = Path("logs/integration_tests/worker_light.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("ab") as log_file:
        subprocess.Popen(
            [
                "uv",
                "run",
                "uvicorn",
                "worker_light.app:app",
                "--host",
                "0.0.0.0",
                "--port",
                "18001",
            ],
            cwd=Path.cwd(),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    await _wait_for_worker_light_health()


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


def _service_debug_log_path(service: str) -> Path:
    candidates = [
        Path(f"logs/integration_tests/current/{service}_debug.log"),
        Path(f"logs/integration_tests/{service}_debug.log"),
        Path(f"logs/{service}_debug.log"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _slim_episode(episode: dict) -> dict:
    return {k: v for k, v in episode.items() if k != "traces"}


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
        episode = await wait_for_episode_terminal(
            client,
            episode_id,
            timeout_s=600.0,
        )

    assert episode["status"] != "FAILED", f"Episode failed: {_slim_episode(episode)}"

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

        await asyncio.gather(
            seed_benchmark_assembly_definition(client, session_a),
            seed_benchmark_assembly_definition(client, session_b),
            seed_execution_reviewer_handover(
                client,
                session_id=session_a,
                int_id="INT-182",
            ),
            seed_execution_reviewer_handover(
                client,
                session_id=session_b,
                int_id="INT-182",
            ),
        )

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
            wait_for_episode_terminal(client, episode_a_id, timeout_s=600.0),
            wait_for_episode_terminal(client, episode_b_id, timeout_s=600.0),
        )

        assert episode_a["status"] != "FAILED", (
            f"Episode A failed: {_slim_episode(episode_a)}"
        )
        assert episode_b["status"] != "FAILED", (
            f"Episode B failed: {_slim_episode(episode_b)}"
        )

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
    controller_debug_log = _service_debug_log_path("controller")
    controller_debug_start_offset = (
        controller_debug_log.stat().st_size if controller_debug_log.exists() else 0
    )
    worker_light_debug_log = _service_debug_log_path("worker_light")
    worker_debug_start_offset = (
        worker_light_debug_log.stat().st_size if worker_light_debug_log.exists() else 0
    )

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
        episode = await wait_for_episode_terminal(
            client,
            episode_id,
            timeout_s=600.0,
        )
        assert episode["status"] != "FAILED", (
            f"Episode failed: {_slim_episode(episode)}"
        )

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

    controller_debug_segment = strip_ansi(
        read_log_segment(controller_debug_log, controller_debug_start_offset)
    )
    assert any(
        "agent_role" in line and AgentName.ENGINEER_CODER.value in line
        for line in controller_debug_segment.splitlines()
    ), controller_debug_segment

    worker_debug_segment = strip_ansi(
        read_log_segment(worker_light_debug_log, worker_debug_start_offset)
    )
    assert any(
        "x-agent-role" in line and AgentName.ENGINEER_CODER.value in line
        for line in worker_debug_segment.splitlines()
    ), worker_debug_segment


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_185_agent_failed_tool_error_routes_and_run_continues():
    """INT-185: Agent-caused tool error is observed, no infra retry fan-out, and run continues."""
    worker_light_debug_log = Path("logs/worker_light_debug.log")
    worker_debug_start_offset = (
        worker_light_debug_log.stat().st_size if worker_light_debug_log.exists() else 0
    )

    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-185",
            task="INT-185 agent failed tool routing contract",
            agent_name=AgentName.ENGINEER_CODER,
        )
        episode = await wait_for_episode_terminal(
            client,
            episode_id,
            timeout_s=300.0,
        )

    assert episode["status"] != "FAILED", f"Episode failed: {_slim_episode(episode)}"
    traces = episode.get("traces", [])
    failed_agent_commands = [
        (idx, t)
        for idx, t in enumerate(traces)
        if t.get("trace_type") == "TOOL_START"
        and t.get("name") == "execute_command"
        and "ls /non_existent_path_to_fail" in (t.get("content") or "")
    ]
    assert failed_agent_commands, (
        "Expected at least one deterministic agent-caused command failure."
    )
    failed_idx = failed_agent_commands[0][0]
    subsequent_tool_starts = [
        idx
        for idx, t in enumerate(traces)
        if idx > failed_idx and t.get("trace_type") == "TOOL_START"
    ]
    assert subsequent_tool_starts, (
        "Expected subsequent tool calls after the failed tool observation."
    )

    worker_log_segment = strip_ansi(
        read_log_segment(worker_light_debug_log, worker_debug_start_offset)
    )
    assert f"session_id={session_id}" in worker_log_segment, worker_log_segment
    assert "runtime_execute_async_complete" in worker_log_segment, worker_log_segment
    assert "exit_code=2" in worker_log_segment, worker_log_segment


@pytest.mark.integration_agent
@pytest.mark.integration_p1
@pytest.mark.allow_backend_errors(
    "SYSTEM_TOOL_RETRY_EXHAUSTED|system_tool_retry_attempt|node_entry_validation_rejected|missing_artifact|reviewer_entry_blocked|connection refused|ConnectError|All connection attempts failed|engineer_coder_dspy_failed"
)
@pytest.mark.asyncio
async def test_int_186_system_failed_tool_retry_cap_and_terminal_metadata():
    """INT-186: Infra tool failures retry up to 3 attempts then fail closed.

    Exception note: this test intentionally uses a deterministic mock LLM/tool script
    scenario (see `tests/integration/mock_responses/`) because the contract under
    test is a devops regression path (infra transport failure + retry exhaustion), not
    product logic. This is a scoped exception for INT-186 only.
    """
    controller_log = get_controller_log_path()
    start_offset = controller_log.stat().st_size if controller_log.exists() else 0

    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id, episode_id = await run_agent_episode(
            client,
            int_id="INT-186",
            task="INT-186 system-failed retry cap contract",
            agent_name=AgentName.ENGINEER_CODER,
        )
        # Intentional outage injection for devops regression coverage:
        # once execute_command starts, drop worker-light mid-call so the same
        # tool request exercises infra-level retries and terminalization.
        # We detect command start via worker log because episode trace sync can lag.
        worker_light_debug_log = Path("logs/worker_light_debug.log")
        worker_debug_start_offset = (
            worker_light_debug_log.stat().st_size
            if worker_light_debug_log.exists()
            else 0
        )
        saw_execute_start = False
        deadline = asyncio.get_event_loop().time() + 60.0
        while asyncio.get_event_loop().time() < deadline:
            if worker_light_debug_log.exists():
                blob = worker_light_debug_log.read_text(
                    encoding="utf-8", errors="ignore"
                )
                # Integration runner may truncate logs between runs; if so, read from start.
                delta = (
                    blob[worker_debug_start_offset:]
                    if len(blob) >= worker_debug_start_offset
                    else blob
                )
                delta = strip_ansi(delta)
                saw_execute_start = (
                    "runtime_execute_async" in delta
                    and f"session_id={session_id}" in delta
                )
            if saw_execute_start:
                break
            await asyncio.sleep(0.5)
        assert saw_execute_start, (
            "Timed out waiting for worker execute_command start before outage injection."
        )

        pid_lookup = subprocess.run(
            ["lsof", "-ti", "tcp:18001", "-sTCP:LISTEN"],
            capture_output=True,
            text=True,
            check=False,
        )
        if pid_lookup.returncode != 0 or not pid_lookup.stdout.strip():
            pid_lookup = subprocess.run(
                ["pgrep", "-f", r"uvicorn.*18001"],
                capture_output=True,
                text=True,
                check=False,
            )
        assert pid_lookup.returncode == 0 and pid_lookup.stdout.strip(), (
            "Unable to resolve live worker-light PID for outage injection."
        )
        worker_pid = int(pid_lookup.stdout.strip().splitlines()[-1])
        # Force immediate process death so the in-flight tool call experiences
        # transport failure (SIGTERM can shut down gracefully after request completes).
        os.kill(worker_pid, signal.SIGKILL)

        episode = await wait_for_episode_terminal(client, episode_id, timeout_s=240.0)

    await _restart_worker_light_after_outage()

    assert episode["status"] == "FAILED", (
        "Episode must fail closed after system tool retry exhaustion."
    )
    metadata = episode.get("metadata_vars") or {}
    assert metadata.get("terminal_reason") == "SYSTEM_TOOL_RETRY_EXHAUSTED", metadata
    assert metadata.get("failure_class") == "INFRA_DEVOPS_FAILURE", metadata

    log_segment = strip_ansi(read_log_segment(controller_log, start_offset))
    assert "SYSTEM_TOOL_RETRY_EXHAUSTED" in log_segment, (
        "Expected retry exhaustion to be recorded in controller logs."
    )

    attempt_lines = [
        line
        for line in log_segment.splitlines()
        if "system_tool_retry_attempt" in line and session_id in line
    ]
    assert len(attempt_lines) == 3, (
        f"Expected exactly 3 infra retry attempts for session {session_id}, got {len(attempt_lines)}."
    )
