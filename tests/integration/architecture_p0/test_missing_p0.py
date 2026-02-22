import os
import uuid
import asyncio
import time
from pathlib import Path
import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


async def get_bundle(client: httpx.AsyncClient, session_id: str) -> str:
    """Fetch gzipped workspace from light worker and return base64 string."""
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/bundle",
        headers={"X-Session-ID": session_id},
        timeout=60.0,
    )
    assert resp.status_code == 200, f"Failed to get bundle: {resp.text}"
    import base64

    return base64.b64encode(resp.content).decode("utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_benchmark_validation():
    """INT-024: Verify benchmark validation catches intersecting geometry."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-024-{int(time.time())}"

        # Script with intersecting boxes
        script_intersect = """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    # Two overlapping boxes
    with BuildPart() as p1:
        Box(10, 10, 10)
    p1.part.label = "part1"
    p1.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    
    with BuildPart() as p2:
        Box(10, 10, 10)
    p2.part.move(Location((5, 0, 0))) # Significant overlap
    p2.part.label = "part2"
    p2.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    
    return Compound(children=[p1.part, p2.part])
"""
        objectives_content = """
objectives:
  goal_zone: {min: [100.0, 100.0, 100.0], max: [110.0, 110.0, 110.0]}
  forbid_zones: []
  build_zone: {min: [-100.0, -100.0, -100.0], max: [100.0, 100.0, 100.0]}
simulation_bounds: {min: [-200.0, -200.0, -200.0], max: [200.0, 200.0, 200.0]}
moved_object:
  label: "target_box"
  shape: "sphere"
  start_position: [0.0, 0.0, 0.0]
  runtime_jitter: [0.0, 0.0, 0.0]
constraints:
  max_unit_cost: 100.0
  max_weight_g: 1000.0
"""
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200

        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "intersect.py",
                "content": script_intersect,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200

        bundle64 = await get_bundle(client, session_id)

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json={"script_path": "intersect.py", "bundle_base64": bundle64},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        data = resp.json()

        assert not data["success"]
        assert "intersection" in data["message"].lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_benchmark_validation_build_zone():
    """INT-024: Verify benchmark validation catches build zone violations."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-024-bz-{int(time.time())}"

        # 1. Objectives with small build zone
        objectives_content = """
objectives:
  goal_zone:
    min: [100.0, 100.0, 100.0]
    max: [110.0, 110.0, 110.0]
  forbid_zones: []
  build_zone:
    min: [-5.0, -5.0, -5.0]
    max: [5.0, 5.0, 5.0]
simulation_bounds:
    min: [-100.0, -100.0, -100.0]
    max: [100.0, 100.0, 100.0]
moved_object:
    label: "target_box"
    shape: "sphere"
    start_position: [0.0, 0.0, 0.0]
    runtime_jitter: [0.0, 0.0, 0.0]
constraints:
    max_unit_cost: 100.0
    max_weight_g: 100.0
"""
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200

        # 2. Script with part outside build zone
        script_outside = """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    with BuildPart() as p:
        Box(20, 20, 20) # Too big for [-5, 5] build zone
    p.part.label = "big_box"
    p.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p.part
"""
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "big.py", "content": script_outside, "overwrite": True},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200

        bundle64 = await get_bundle(client, session_id)

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json={"script_path": "big.py", "bundle_base64": bundle64},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        data = resp.json()

        assert not data["success"]
        assert "build zone violation" in data["message"].lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_014_cots_propagation():
    """INT-014: Verify COTS data propagates into plan and assembly definition."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-014-{uuid.uuid4().hex[:8]}"
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={
                "task": "Design a mechanism with a servo motor",
                "session_id": session_id,
            },
        )
        assert run_resp.status_code == 202
        episode_id = run_resp.json()["episode_id"]

        # Wait for agent to reach EXECUTING or COMPLETED status (which means planning is done)
        max_attempts = 30
        for _ in range(max_attempts):
            await asyncio.sleep(5.0)
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            data = status_resp.json()
            if data["status"] in ["executing", "completed"]:
                break
        else:
            pytest.fail("Agent did not complete planning in time")

        # Verify plan.md contains COTS ID
        plan_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "plan.md"},
            headers={"X-Session-ID": session_id},
        )
        assert plan_resp.status_code == 200
        plan_content = plan_resp.json()["content"]
        assert "MOCK-MOTOR-ID" in plan_content

        # Verify assembly_definition.yaml contains COTS data
        asm_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "assembly_definition.yaml"},
            headers={"X-Session-ID": session_id},
        )
        assert asm_resp.status_code == 200
        import yaml

        asm_data = yaml.safe_load(asm_resp.json()["content"])
        cots_parts = asm_data.get("cots_parts", [])
        assert any(p["part_id"] == "MOCK-MOTOR-ID" for p in cots_parts)


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_025_events_collection_e2e():
    """INT-025: Verify worker events are ingested and persisted as traces."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-025-{uuid.uuid4().hex[:8]}"
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": "Run a simulation", "session_id": session_id},
        )
        assert run_resp.status_code == 202
        episode_id = run_resp.json()["episode_id"]

        # Wait for completion
        max_attempts = 30
        for _ in range(max_attempts):
            await asyncio.sleep(5.0)
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            data = status_resp.json()
            if data["status"] == "completed":
                break
        else:
            pytest.fail("Agent did not complete simulation task in time")

        # Verify traces contain an event from simulation
        # simulate() tool emits 'simulation_backend_selected' and 'simulation_result' events
        traces = data.get("traces", [])
        event_traces = [t for t in traces if t["trace_type"] == "event"]
        assert len(event_traces) > 0, "No event traces found"

        event_names = [t["name"] for t in event_traces]
        assert (
            "simulation_backend_selected" in event_names
            or "simulation_result" in event_names
        )
