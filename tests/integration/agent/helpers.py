import asyncio
import os
import re
import uuid
from pathlib import Path

import httpx

from shared.enums import AgentName

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def strip_ansi(value: str) -> str:
    return _ANSI_RE.sub("", value)


def get_controller_log_path() -> Path:
    candidates = [
        Path("logs/integration_tests/current/controller.log"),
        Path("logs/integration_tests/controller.log"),
        Path("logs/controller.log"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def read_log_segment(path: Path, start_offset: int) -> str:
    if not path.exists():
        return ""

    with path.open("rb") as f:
        f.seek(start_offset)
        return f.read().decode("utf-8", errors="ignore")


async def run_agent_episode(
    client: httpx.AsyncClient,
    *,
    int_id: str,
    task: str,
    agent_name: AgentName = AgentName.ENGINEER_CODER,
) -> tuple[str, str]:
    session_id = f"{int_id}-{uuid.uuid4().hex[:8]}"
    resp = await client.post(
        f"{CONTROLLER_URL}/api/agent/run",
        json={
            "task": task,
            "session_id": session_id,
            "agent_name": agent_name,
        },
    )
    assert resp.status_code == 202, f"Failed to start agent run: {resp.text}"
    episode_id = resp.json()["episode_id"]
    return session_id, episode_id


async def wait_for_episode_terminal(
    client: httpx.AsyncClient,
    episode_id: str,
    *,
    timeout_s: float = 180.0,
    poll_s: float = 1.0,
) -> dict:
    terminal = {"COMPLETED", "FAILED", "CANCELLED", "PLANNED"}
    deadline = asyncio.get_event_loop().time() + timeout_s

    while asyncio.get_event_loop().time() < deadline:
        resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert resp.status_code == 200, f"Episode lookup failed: {resp.text}"
        payload = resp.json()
        if payload.get("status") in terminal:
            return payload
        await asyncio.sleep(poll_s)

    msg = f"Episode {episode_id} did not reach terminal status in {timeout_s}s"
    raise AssertionError(msg)


async def wait_for_queue_empty(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    timeout_s: float = 60.0,
    poll_s: float = 0.5,
) -> list[dict]:
    deadline = asyncio.get_event_loop().time() + timeout_s

    while asyncio.get_event_loop().time() < deadline:
        resp = await client.get(f"{CONTROLLER_URL}/api/v1/sessions/{session_id}/queue")
        assert resp.status_code == 200, f"Queue lookup failed: {resp.text}"
        queued = resp.json()
        if not queued:
            return queued
        await asyncio.sleep(poll_s)

    msg = f"Queue for session {session_id} did not drain in {timeout_s}s"
    raise AssertionError(msg)
