import asyncio
import hashlib
import os
import re
import time
import uuid
from pathlib import Path

import httpx
import yaml

from shared.enums import AgentName
from shared.models.schemas import AssemblyConstraints, AssemblyDefinition, CostTotals
from shared.models.simulation import SimulationResult
from shared.workers.schema import (
    ReviewManifest,
    ValidationResultRecord,
    WriteFileRequest,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")

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


def _benchmark_assembly_definition_content() -> str:
    assembly = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=200.0,
            benchmark_max_weight_g=1000.0,
            planner_target_max_unit_cost_usd=200.0,
            planner_target_max_weight_g=500.0,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[],
        totals=CostTotals(
            estimated_unit_cost_usd=0.0,
            estimated_weight_g=0.0,
            estimate_confidence="medium",
        ),
    )
    return yaml.safe_dump(
        assembly.model_dump(mode="json", by_alias=True),
        sort_keys=False,
    )


def _fixture_script_content(int_id: str) -> str:
    responses_path = Path("tests/integration/mock_responses.yaml")
    responses = yaml.safe_load(responses_path.read_text(encoding="utf-8")) or {}
    scenarios = responses.get("scenarios") or {}
    scenario = scenarios.get(int_id)
    if not scenario:
        fallback_path = (
            Path("tests/integration/mock_responses")
            / int_id
            / "engineer_coder"
            / "entry_01"
            / "01__script.py"
        )
        return fallback_path.read_text(encoding="utf-8")

    for node_block in scenario.get("transcript", []):
        if node_block.get("node") != "engineer_coder":
            continue
        for step in node_block.get("steps", []):
            tool_args = step.get("tool_args") or {}
            if (
                step.get("tool_name") == "write_file"
                and tool_args.get("path") == "script.py"
            ):
                content = tool_args.get("content")
                if isinstance(content, str):
                    return content

    fallback_path = (
        Path("tests/integration/mock_responses")
        / int_id
        / "engineer_coder"
        / "entry_01"
        / "01__script.py"
    )
    return fallback_path.read_text(encoding="utf-8")


async def _seed_workspace_file(
    client: httpx.AsyncClient,
    *,
    session_id: str,
    path: str,
    content: str,
    bypass_agent_permissions: bool = False,
) -> None:
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path=path,
            content=content,
            overwrite=True,
            bypass_agent_permissions=bypass_agent_permissions,
        ).model_dump(),
        headers={
            "X-Session-ID": session_id,
            **({"X-System-FS-Bypass": "1"} if bypass_agent_permissions else {}),
        },
    )
    assert resp.status_code == 200, resp.text


async def seed_benchmark_assembly_definition(
    client: httpx.AsyncClient,
    session_id: str,
) -> None:
    """Seed a minimal benchmark handoff file for engineer planner startup."""
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="benchmark_assembly_definition.yaml",
        content=_benchmark_assembly_definition_content(),
    )


async def seed_execution_reviewer_handover(
    client: httpx.AsyncClient,
    *,
    session_id: str,
    int_id: str,
) -> None:
    """Seed deterministic reviewer handoff artifacts for execution-reviewer runs."""
    script_content = _fixture_script_content(int_id)
    script_sha256 = hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    seed_ts = time.time()

    validation_record = ValidationResultRecord(
        success=True,
        message="Validation completed",
        timestamp=seed_ts,
        script_path="script.py",
        script_sha256=script_sha256,
    )
    simulation_result = SimulationResult(
        success=True,
        summary="Goal achieved in green zone.",
        render_paths=[],
        confidence="high",
    )
    review_manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage="engineering_execution_reviewer",
        session_id=session_id,
        script_path="script.py",
        script_sha256=script_sha256,
        validation_success=True,
        validation_timestamp=seed_ts,
        simulation_success=True,
        simulation_summary="Goal achieved in green zone.",
        simulation_timestamp=seed_ts,
        goal_reached=True,
        renders=[],
        mjcf_path="renders/scene.xml",
        cad_path="renders/model.step",
        objectives_path="renders/benchmark_definition.yaml",
        assembly_definition_path="renders/assembly_definition.yaml",
    )

    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="validation_results.json",
        content=validation_record.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="simulation_result.json",
        content=simulation_result.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path=".manifests/engineering_execution_review_manifest.json",
        content=review_manifest.model_dump_json(indent=2),
        bypass_agent_permissions=True,
    )


async def run_agent_episode(
    client: httpx.AsyncClient,
    *,
    int_id: str,
    task: str,
    agent_name: AgentName = AgentName.ENGINEER_CODER,
) -> tuple[str, str]:
    session_id = f"{int_id}-{uuid.uuid4().hex[:8]}"

    if int_id in {"INT-181", "INT-182", "INT-183"}:
        await seed_benchmark_assembly_definition(client, session_id)
    if int_id in {"INT-181", "INT-182", "INT-183"}:
        await seed_execution_reviewer_handover(
            client,
            session_id=session_id,
            int_id=int_id,
        )

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
