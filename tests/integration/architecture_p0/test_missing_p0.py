import asyncio
import os
import uuid

import httpx
import pytest

from controller.api.schemas import (
    AgentRunResponse,
    EpisodeListItem,
    EpisodeResponse,
    EpisodeCreateResponse,
)
from controller.api.tasks import AgentRunRequest
from shared.enums import EpisodeStatus
from shared.workers.schema import (
    ReadFileRequest,
    ReadFileResponse,
    WriteFileRequest,
)

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_004_episode_artifact_persistence():
    """INT-004: Verify artifacts are persisted and accessible via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-p0-{uuid.uuid4().hex[:8]}"

        # Write some files
        write_req = WriteFileRequest(path="output.py", content="print('hello')")
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # Create dummy episode
        req = AgentRunRequest(
            task="Test artifacts",
            session_id=session_id,
            metadata_vars={"worker_session_id": session_id},
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/test/episodes",
            json=req.model_dump(mode="json"),
        )
        assert resp.status_code == 201
        episode_create = EpisodeCreateResponse.model_validate(resp.json())
        episode_id = str(episode_create.episode_id)

        # Let's check episodes list
        resp = await client.get(f"{CONTROLLER_URL}/episodes/")
        assert resp.status_code == 200
        episodes = [EpisodeListItem.model_validate(e) for e in resp.json()]
        assert any(str(e.id) == episode_id for e in episodes)


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_005_trace_realtime_broadcast():
    """INT-005: Verify traces are broadcasted via DB/API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-trace-{uuid.uuid4().hex[:8]}"

        # Run a very short agent task
        run_req = AgentRunRequest(task="Say hello", session_id=session_id)
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=run_req.model_dump(mode="json"),
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        # Wait a bit for traces
        await asyncio.sleep(5.0)

        resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        ep_data = EpisodeResponse.model_validate(resp.json())
        assert len(ep_data.traces) > 0


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_011_planner_target_caps_validation():
    """INT-011: Verify planner target caps must be <= benchmark caps."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-caps-{uuid.uuid4().hex[:8]}"
        headers = {"X-Session-ID": session_id}

        # 1. Setup required files for submission
        plan_md = """## 1. Solution Overview\nWe will build a box.\n\n## 2. Parts List\n- Box: 10x10x10mm\n\n## 3. Assembly Strategy\n1. Build the box.\n\n## 4. Cost & Weight Budget\n- Cost: $10\n\n## 5. Risk Assessment\n- None."""
        todo_md = "- [x] Task 1"
        objectives_yaml = """
objectives:
  goal_zone:
    min: [10, 10, 10]
    max: [20, 20, 20]
  build_zone:
    min: [0, 0, 0]
    max: [100, 100, 100]
simulation_bounds:
  min: [-10, -10, -10]
  max: [110, 110, 110]
moved_object:
  label: target_box
  shape: box
  start_position: [0, 0, 5]
  runtime_jitter: [0, 0, 0]
constraints:
  max_unit_cost: 100.0
  max_weight_g: 1000.0
"""
        script_py = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return p
"""
        # Note: we need to initialize a git repo for immutability check to pass or be skipped cleanly
        await client.post(f"{WORKER_LIGHT_URL}/git/init", headers=headers)

        files = {
            "plan.md": plan_md,
            "todo.md": todo_md,
            "objectives.yaml": objectives_yaml,
            "script.py": script_py,
            "validation_results.json": '{"is_valid": true}',
        }

        for path, content in files.items():
            await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=WriteFileRequest(path=path, content=content).model_dump(mode="json"),
                headers=headers,
            )

        # 2. Write INVALID assembly_definition.yaml (planner target > benchmark cap)
        invalid_asm = """
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 100.0
  benchmark_max_weight_g: 1000.0
  planner_target_max_unit_cost_usd: 150.0  # INVALID (> 100.0)
  planner_target_max_weight_g: 500.0
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: high
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(path="assembly_definition.yaml", content=invalid_asm).model_dump(mode="json"),
            headers=headers,
        )

        # 3. Call /benchmark/submit
        from shared.workers.schema import BenchmarkToolRequest, BenchmarkToolResponse
        submit_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=submit_req.model_dump(mode="json"),
            headers=headers,
            timeout=60.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "Planner target cost" in data.message
        assert "(150.0)" in data.message
        assert "benchmark max cost" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_014_cots_propagation():
    """INT-014: Verify COTS data propagates into plan and assembly definition."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-014-{uuid.uuid4().hex[:8]}"
        run_req = AgentRunRequest(
            task="Design a mechanism with a servo motor",
            session_id=session_id,
        )
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=run_req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202
        run_data = AgentRunResponse.model_validate(run_resp.json())
        episode_id = run_data.episode_id

        # Wait for agent to reach COMPLETED status
        max_attempts = 60
        for _ in range(max_attempts):
            await asyncio.sleep(5.0)
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            ep_data = EpisodeResponse.model_validate(status_resp.json())
            if ep_data.status == EpisodeStatus.COMPLETED:
                break
        else:
            pytest.fail("Agent did not complete planning in time")

        # Verify plan.md contains COTS ID
        read_plan_req = ReadFileRequest(path="plan.md")
        plan_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=read_plan_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert plan_resp.status_code == 200
        plan_data = ReadFileResponse.model_validate(plan_resp.json())
        assert "MOCK-MOTOR-ID" in plan_data.content

        # Verify assembly_definition.yaml contains COTS data
        read_asm_req = ReadFileRequest(path="assembly_definition.yaml")
        asm_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=read_asm_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert asm_resp.status_code == 200
        asm_data = ReadFileResponse.model_validate(asm_resp.json())
        assert "MOCK-MOTOR-ID" in asm_data.content


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_025_events_collection_e2e():
    """INT-025: Verify worker events are ingested and persisted as traces."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-025-{uuid.uuid4().hex[:8]}"
        run_req = AgentRunRequest(task="Run a simulation", session_id=session_id)
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=run_req.model_dump(mode="json"),
        )
        assert run_resp.status_code == 202
        run_data = AgentRunResponse.model_validate(run_resp.json())
        episode_id = run_data.episode_id

        # Wait for completion
        max_attempts = 60
        for _ in range(max_attempts):
            await asyncio.sleep(5.0)
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code != 200:
                continue
            ep_data = EpisodeResponse.model_validate(status_resp.json())
            if ep_data.status == EpisodeStatus.COMPLETED:
                break
        else:
            pytest.fail("Agent did not complete simulation task in time")

        # Verify traces contain the simulation result event
        resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        ep_data = EpisodeResponse.model_validate(resp.json())

        assert any("simulation" in str(t.content).lower() for t in ep_data.traces)
