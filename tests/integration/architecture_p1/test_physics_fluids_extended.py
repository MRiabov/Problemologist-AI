import asyncio
import os
import time

import httpx
import pytest

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:18001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_138_smoke_test_mode():
    """INT-138: Verify smoke-test mode for Genesis."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-138-{int(time.time())}"

        objectives_content = """
physics:
  backend: "genesis"
objectives:
  goal_zone: {min: [10,10,10], max: [12,12,12]}
  build_zone: {min: [-100,-100,-100], max: [100,100,100]}
simulation_bounds: {min: [-100,-100,-100], max: [100,100,100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 100, max_weight: 10}
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "script.py",
                "content": "from build123d import *; def build(): return Box(1,1,1)",
            },
            headers={"X-Session-ID": session_id},
        )

        # Trigger simulation with smoke_test_mode=True
        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # Verify result labelled approximate
        assert (
            data["confidence"] == "approximate"
        ), f"Expected confidence 'approximate', got '{data['confidence']}'"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_139_fluid_storage_policy():
    """INT-139: Verify fluid data storage policy."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-139-{int(time.time())}"

        objectives_content = """
physics:
  backend: "genesis"
objectives:
  goal_zone: {min: [10,10,10], max: [12,12,12]}
  build_zone: {min: [-100,-100,-100], max: [100,100,100]}
simulation_bounds: {min: [-100,-100,-100], max: [100,100,100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 100, max_weight: 10}
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "script.py",
                "content": "from build123d import *; def build(): return Box(1,1,1)",
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={"script_path": "script.py"},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200

        # Verify artifacts in response (MP4, JSON, etc.)
        data = resp.json()
        artifacts = data.get("artifacts", {})
        assert "render_paths" in artifacts

        # Verify raw particle data absent from workspace
        ls_resp = await client.post(
            f"{WORKER_URL}/fs/ls", json={"path": "/"}, headers={"X-Session-ID": session_id}
        )
        files = [f["name"] for f in ls_resp.json()]
        assert not any(
            "particles" in f.lower() for f in files
        ), "Raw particle data found in workspace"
