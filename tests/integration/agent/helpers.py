import asyncio
import hashlib
import json
import os
import re
import time
import uuid
from collections.abc import Awaitable, Callable
from functools import lru_cache
from pathlib import Path

import httpx
import yaml
from websockets.asyncio.client import connect as websocket_connect

from controller.agent.mock_scenarios import load_integration_mock_scenarios
from controller.api.schemas import EpisodeResponse
from shared.enums import AgentName, EpisodeStatus
from shared.git_utils import repo_revision
from shared.models.schemas import AssemblyConstraints, AssemblyDefinition, CostTotals
from shared.models.simulation import MultiRunResult, SimulationMetrics, SimulationResult
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
    ReviewManifest,
    ValidationResultRecord,
    WriteFileRequest,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
REPO_MANUFACTURING_CONFIG = Path(
    "worker_heavy/workbenches/manufacturing_config.yaml"
).read_text(encoding="utf-8")

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


def _controller_ws_url(path: str) -> str:
    if CONTROLLER_URL.startswith("https://"):
        return f"wss://{CONTROLLER_URL.removeprefix('https://').rstrip('/')}{path}"
    return f"ws://{CONTROLLER_URL.removeprefix('http://').rstrip('/')}{path}"


async def _fetch_episode(client: httpx.AsyncClient, episode_id: str) -> EpisodeResponse:
    response = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
    assert response.status_code == 200, response.text
    return EpisodeResponse.model_validate(response.json())


async def _fetch_benchmark_session(
    client: httpx.AsyncClient,
    session_id: str,
) -> EpisodeResponse | None:
    response = await client.get(f"{CONTROLLER_URL}/benchmark/{session_id}")
    if response.status_code == 404:
        return None
    assert response.status_code == 200, response.text
    return EpisodeResponse.model_validate(response.json())


def _benchmark_assembly_definition_content(
    *,
    benchmark_max_unit_cost_usd: float = 200.0,
    benchmark_max_weight_g: float = 1000.0,
    planner_target_max_unit_cost_usd: float | None = None,
    planner_target_max_weight_g: float | None = None,
    estimated_unit_cost_usd: float = 0.0,
    estimated_weight_g: float = 0.0,
    estimate_confidence: str = "medium",
) -> str:
    planner_target_max_unit_cost_usd = (
        benchmark_max_unit_cost_usd
        if planner_target_max_unit_cost_usd is None
        else planner_target_max_unit_cost_usd
    )
    planner_target_max_weight_g = (
        benchmark_max_weight_g
        if planner_target_max_weight_g is None
        else planner_target_max_weight_g
    )
    assembly = AssemblyDefinition(
        version="1.0",
        constraints=AssemblyConstraints(
            planner_target_max_unit_cost_usd=planner_target_max_unit_cost_usd,
            planner_target_max_weight_g=planner_target_max_weight_g,
        ),
        manufactured_parts=[],
        cots_parts=[],
        final_assembly=[],
        totals=CostTotals(
            estimated_unit_cost_usd=estimated_unit_cost_usd,
            estimated_weight_g=estimated_weight_g,
            estimate_confidence=estimate_confidence,
        ),
    )
    return yaml.safe_dump(
        assembly.model_dump(mode="json", by_alias=True, exclude_none=True),
        sort_keys=False,
    )


@lru_cache(maxsize=1)
def _integration_mock_scenarios() -> dict[str, dict[str, object]]:
    return load_integration_mock_scenarios()


def _fixture_script_content(int_id: str) -> str:
    scenario = _integration_mock_scenarios().get(int_id)
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


def repo_git_revision() -> str:
    revision = repo_revision(Path(__file__).resolve().parents[3])
    assert revision, "repository git revision could not be determined."
    return revision


def integration_workspace_session_id(task: str, session_id: str) -> str:
    """Return the worker workspace session id used by integration runs."""
    return session_id


async def seed_benchmark_assembly_definition(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    benchmark_max_unit_cost_usd: float = 200.0,
    benchmark_max_weight_g: float = 1000.0,
    planner_target_max_unit_cost_usd: float | None = None,
    planner_target_max_weight_g: float | None = None,
    estimated_unit_cost_usd: float = 0.0,
    estimated_weight_g: float = 0.0,
    estimate_confidence: str = "medium",
) -> None:
    """Seed a minimal benchmark handoff file for engineer planner startup."""
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="benchmark_assembly_definition.yaml",
        content=_benchmark_assembly_definition_content(
            benchmark_max_unit_cost_usd=benchmark_max_unit_cost_usd,
            benchmark_max_weight_g=benchmark_max_weight_g,
            planner_target_max_unit_cost_usd=planner_target_max_unit_cost_usd,
            planner_target_max_weight_g=planner_target_max_weight_g,
            estimated_unit_cost_usd=estimated_unit_cost_usd,
            estimated_weight_g=estimated_weight_g,
            estimate_confidence=estimate_confidence,
        ),
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="manufacturing_config.yaml",
        content=REPO_MANUFACTURING_CONFIG,
    )


async def seed_execution_reviewer_handover(
    client: httpx.AsyncClient,
    *,
    session_id: str,
    int_id: str,
    script_content: str | None = None,
    render_path: str = "renders/render_e45_a45.png",
    seed_render_preview: bool = True,
) -> None:
    """Seed deterministic reviewer handoff artifacts for execution-reviewer runs."""
    script_content = script_content or _fixture_script_content(int_id)
    script_sha256 = hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    seed_ts = time.time()
    revision = repo_git_revision()
    render_base = Path(render_path).with_suffix("")
    render_rgb_path = render_path
    render_depth_path = f"{render_base}_depth.png"
    render_segmentation_path = f"{render_base}_segmentation.png"
    benchmark_definition_seed = (
        "version: 1.0\n"
        "constraints:\n"
        "  benchmark_max_unit_cost_usd: 200.0\n"
        "  benchmark_max_weight_g: 1000.0\n"
    )
    assembly_definition_seed = (
        "version: 1.0\n"
        "constraints:\n"
        "  planner_target_max_unit_cost_usd: 200.0\n"
        "  planner_target_max_weight_g: 1000.0\n"
    )

    validation_record = ValidationResultRecord(
        success=True,
        message="Validation completed",
        timestamp=seed_ts,
        script_path="script.py",
        script_sha256=script_sha256,
        verification_result=MultiRunResult(
            num_scenes=1,
            success_count=1,
            success_rate=1.0,
            is_consistent=True,
            individual_results=[SimulationMetrics(success=True)],
            fail_reasons=[],
            scene_build_count=1,
            backend_run_count=1,
            batched_execution=True,
        ),
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
        revision=revision,
        renders=[
            render_rgb_path,
            render_depth_path,
            render_segmentation_path,
        ],
        mjcf_path="renders/scene.xml",
        cad_path="renders/model.step",
        objectives_path="renders/benchmark_definition.yaml",
        assembly_definition_path="renders/assembly_definition.yaml",
    )

    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="script.py",
        content=script_content,
        bypass_agent_permissions=True,
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
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="renders/scene.xml",
        content="<mujoco><worldbody /></mujoco>\n",
        bypass_agent_permissions=True,
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="renders/model.step",
        content="ISO-10303-21;\nEND-ISO-10303-21;\n",
        bypass_agent_permissions=True,
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="renders/benchmark_definition.yaml",
        content=benchmark_definition_seed,
        bypass_agent_permissions=True,
    )
    await _seed_workspace_file(
        client,
        session_id=session_id,
        path="renders/assembly_definition.yaml",
        content=assembly_definition_seed,
        bypass_agent_permissions=True,
    )
    if seed_render_preview:
        await seed_current_revision_render_preview(
            client,
            session_id=session_id,
            render_path=render_path,
        )


async def seed_current_revision_render_preview(
    client: httpx.AsyncClient,
    *,
    session_id: str,
    render_path: str = "renders/render_e45_a45.png",
) -> None:
    """Seed a current-revision render preview image and manifest for reviewer tests."""

    revision = repo_git_revision()
    render_base = Path(render_path).with_suffix("")
    image_name = Path(render_path).name
    depth_name = f"{render_base.name}_depth.png"
    segmentation_name = f"{render_base.name}_segmentation.png"
    group_key = Path(render_path).stem
    manifest = RenderManifest(
        version="1.0",
        episode_id=session_id,
        worker_session_id=session_id,
        revision=revision,
        environment_version="integration-test",
        preview_evidence_paths=[
            render_path,
            f"{render_base}_depth.png",
            f"{render_base}_segmentation.png",
        ],
        artifacts={
            render_path: RenderArtifactMetadata(
                modality="rgb",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=render_path,
                    depth=f"{render_base}_depth.png",
                    segmentation=f"{render_base}_segmentation.png",
                ),
            ),
            f"{render_base}_depth.png": RenderArtifactMetadata(
                modality="depth",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=render_path,
                    depth=f"{render_base}_depth.png",
                    segmentation=f"{render_base}_segmentation.png",
                ),
                depth_interpretation=(
                    "Brighter pixels are nearer. Values are normalized per image "
                    "from the build123d/VTK preview renderer."
                ),
            ),
            f"{render_base}_segmentation.png": RenderArtifactMetadata(
                modality="segmentation",
                group_key=group_key,
                siblings=RenderSiblingPaths(
                    rgb=render_path,
                    depth=f"{render_base}_depth.png",
                    segmentation=f"{render_base}_segmentation.png",
                ),
            ),
        },
    )
    code = f"""
python3 - <<'PY'
from pathlib import Path

from PIL import Image

root = Path("renders")
root.mkdir(parents=True, exist_ok=True)
image_path = root / {image_name!r}
depth_path = root / {depth_name!r}
segmentation_path = root / {segmentation_name!r}
Image.new("RGB", (1, 1), (255, 0, 0)).save(image_path)
Image.new("RGB", (1, 1), (0, 255, 0)).save(depth_path)
Image.new("RGB", (1, 1), (0, 0, 255)).save(segmentation_path)
(root / "render_manifest.json").write_text({manifest.model_dump_json(indent=2)!r}, encoding="utf-8")
PY
"""
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/runtime/execute",
        json={
            "code": code,
            "timeout": 30,
            "episode_id": session_id,
        },
        headers={"X-Session-ID": session_id},
    )
    assert resp.status_code == 200, resp.text


async def run_agent_episode(
    client: httpx.AsyncClient,
    *,
    int_id: str,
    task: str,
    agent_name: AgentName = AgentName.ENGINEER_CODER,
) -> tuple[str, str]:
    session_id = f"{int_id}-{uuid.uuid4().hex[:8]}"
    workspace_session_id = integration_workspace_session_id(task, session_id)

    if int_id in {"INT-181", "INT-182", "INT-183", "INT-185", "INT-186"}:
        await seed_benchmark_assembly_definition(client, workspace_session_id)
    if int_id in {"INT-181", "INT-182", "INT-183", "INT-185", "INT-186"}:
        await seed_execution_reviewer_handover(
            client,
            session_id=workspace_session_id,
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
    terminal_statuses: set[EpisodeStatus | str] | None = None,
) -> dict:
    return await wait_for_episode_state(
        client,
        episode_id,
        timeout_s=timeout_s,
        poll_s=poll_s,
        terminal_statuses=terminal_statuses,
    )


async def wait_for_episode_state(
    client: httpx.AsyncClient,
    episode_id: str,
    *,
    timeout_s: float = 180.0,
    poll_s: float = 1.0,
    terminal_statuses: set[EpisodeStatus | str] | None = None,
    predicate: Callable[[EpisodeResponse], bool] | None = None,
) -> dict:
    return await _wait_for_resource_state(
        fetch_resource=lambda: _fetch_episode(client, episode_id),
        ws_path=f"/api/episodes/{episode_id}/ws",
        timeout_s=timeout_s,
        poll_s=poll_s,
        terminal_statuses=terminal_statuses,
        predicate=predicate,
    )


async def wait_for_benchmark_state(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    timeout_s: float = 180.0,
    poll_s: float = 1.0,
    terminal_statuses: set[EpisodeStatus | str] | None = None,
    predicate: Callable[[EpisodeResponse], bool] | None = None,
) -> dict:
    return await _wait_for_resource_state(
        fetch_resource=lambda: _fetch_benchmark_session(client, session_id),
        ws_path=f"/benchmark/{session_id}/ws",
        timeout_s=timeout_s,
        poll_s=poll_s,
        terminal_statuses=terminal_statuses,
        predicate=predicate,
    )


async def _wait_for_resource_state(
    *,
    fetch_resource: Callable[[], Awaitable[EpisodeResponse | None]],
    ws_path: str,
    timeout_s: float = 180.0,
    poll_s: float = 1.0,
    terminal_statuses: set[EpisodeStatus | str] | None = None,
    predicate: Callable[[EpisodeResponse], bool] | None = None,
) -> dict:
    terminal_source = (
        {"COMPLETED", "FAILED", "CANCELLED", "PLANNED"}
        if terminal_statuses is None
        else terminal_statuses
    )
    terminal = {
        status.value if isinstance(status, EpisodeStatus) else str(status)
        for status in terminal_source
    }
    deadline = asyncio.get_running_loop().time() + timeout_s

    resource = await fetch_resource()
    if resource is not None and (
        (predicate is not None and predicate(resource))
        or resource.status.value in terminal
    ):
        return resource.model_dump(mode="json")

    ws_url = _controller_ws_url(ws_path)
    try:
        async with websocket_connect(ws_url, open_timeout=min(10.0, timeout_s)) as ws:
            while asyncio.get_running_loop().time() < deadline:
                remaining = deadline - asyncio.get_running_loop().time()
                if remaining <= 0:
                    break
                recv_timeout = min(remaining, max(1.0, poll_s))
                try:
                    raw_message = await asyncio.wait_for(
                        ws.recv(), timeout=recv_timeout
                    )
                except asyncio.TimeoutError:
                    resource = await fetch_resource()
                    if resource is None:
                        continue
                    if predicate is not None and predicate(resource):
                        return resource.model_dump(mode="json")
                    if resource.status.value in terminal:
                        return resource.model_dump(mode="json")
                    continue
                try:
                    payload = (
                        json.loads(raw_message)
                        if isinstance(raw_message, str)
                        else raw_message
                    )
                except Exception:
                    continue
                if not isinstance(payload, dict):
                    continue
                if payload.get("type") != "status_update":
                    continue
                resource = await fetch_resource()
                if resource is None:
                    continue
                if predicate is not None and predicate(resource):
                    return resource.model_dump(mode="json")
                if resource.status.value in terminal:
                    return resource.model_dump(mode="json")
    except Exception:
        pass

    while asyncio.get_running_loop().time() < deadline:
        resource = await fetch_resource()
        if resource is not None and (
            (predicate is not None and predicate(resource))
            or resource.status.value in terminal
        ):
            return resource.model_dump(mode="json")
        await asyncio.sleep(poll_s)

    msg = f"Resource at {ws_path} did not reach target state in {timeout_s}s"
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
