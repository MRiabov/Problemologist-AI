import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


@pytest.fixture
def session_id():
    return f"test-gpu-{int(time.time())}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_110_gpu_oom_retry(session_id, base_headers):
    """
    INT-110: GPU OOM retry with particle reduction.
    Note: We likely need to MOCK the OOM error unless we have a real GPU
    and can force it. Since integration tests should run in CI,
    we'll use a trigger or a simulated OOM if supported by the backend wrapper.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Forcing OOM is hard. We'll check if the code handles it by
        # injecting a failure in the simulation loop or using a 'bomb' particle count.

        objectives_content = """
physics:
  backend: "genesis"
  compute_target: "cuda"
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-10, -10, -10], max: [10, 10, 10]}
simulation_bounds: {min: [-10, -10, -10], max: [10, 10, 10]}
moved_object: {label: "obj", shape: "sphere", start_position: [0, 0, 5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        # If the environment has NO GPU, this might fail differently.
        # But INT-110 says "Requires GPU environment; currently not tested."
        # I will implement it as a "known-to-fail-or-skip-if-no-gpu" test.

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content, "overwrite": True},
            headers=base_headers,
        )

        # Write a minimal script.py
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.enums import ManufacturingMethod
def build():
    part = Box(1, 1, 1)
    part.metadata = PartMetadata(manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum_6061")
    part.label = "obj"
    return part
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "script.py", "content": script_content},
            headers=base_headers,
        )

        # Write trigger file for Mock OOM
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": ".mock_oom", "content": "trigger"},
            headers=base_headers,
        )

        # Trigger simulation with a huge particle count that exceeds typical RAM/VRAM
        # (Though current worker caps at 100k, we might need a way to force OOM)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "particle_budget": 10**9},
            headers=base_headers,
            timeout=180.0,
        )

        # If it retried, we expect 'gpu_oom_retry' event
        # and result annotated 'confidence: approximate'
        data = resp.json()
        assert resp.status_code == 200, f"Simulation failed: {data.get('message')}"

        # In our mock environment, it SHOULD have retried
        assert any(e.get("event_type") == "gpu_oom_retry" for e in data.get("events", [])), "OOM retry not triggered"
        assert data.get("confidence") == "approximate"
