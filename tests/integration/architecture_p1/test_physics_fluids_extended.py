import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
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
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )

        script_content = """from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1,1,1)
    b.metadata = PartMetadata(material_id="aluminum_6061")
    return b
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "script.py",
                "content": script_content,
            },
            headers={"X-Session-ID": session_id},
        )

        # Trigger simulation with smoke_test_mode=True
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        from shared.workers.schema import BenchmarkToolResponse

        data = BenchmarkToolResponse.model_validate(resp.json())

        # Verify result labelled approximate
        assert data.confidence == "approximate", (
            f"Expected confidence 'approximate', got '{data.confidence}'"
        )


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
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )

        script_content = """from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1,1,1)
    b.metadata = PartMetadata(material_id="aluminum_6061")
    return b
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "script.py",
                "content": script_content,
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/simulate",
            json={"script_path": "script.py"},
            headers={"X-Session-ID": session_id},
            timeout=120.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        # Verify artifacts in response (MP4, JSON, etc.)
        assert data.success, f"Simulation failed: {data.message}"
        artifacts = data.artifacts
        assert artifacts is not None, (
            f"Artifacts missing in response. Message: {data.message}"
        )
        assert "render_paths" in artifacts, (
            f"render_paths missing in artifacts. Keys: {list(artifacts.keys())}"
        )

        # Verify raw particle data absent from workspace
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json={"path": "/"},
            headers={"X-Session-ID": session_id},
        )
        files = [f["name"] for f in ls_resp.json()]
        assert not any("particles" in f.lower() for f in files), (
            "Raw particle data found in workspace"
        )
