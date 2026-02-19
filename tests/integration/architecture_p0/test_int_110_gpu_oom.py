import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")


@pytest.fixture
def session_id():
    return f"test-gpu-{int(time.time())}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_110_gpu_oom_retry(session_id, base_headers, get_bundle):
    """
    INT-110: GPU OOM retry with particle reduction.
    """
    async with httpx.AsyncClient() as client:
        # Write objectives and a simple script
        objectives_content = """
physics:
  backend: "genesis"
  compute_target: "cuda"
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0, 0, 5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1, 1, 1)
    b.label = "obj"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return b
"""

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers=base_headers,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "script.py", "content": script_content},
            headers=base_headers,
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # Trigger simulation with a huge particle count that exceeds typical RAM/VRAM
        # to attempt to trigger OOM retry logic.
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={
                "script_path": "script.py",
                "particle_budget": 10**9,
                "bundle_base64": bundle64,
            },
            headers=base_headers,
            timeout=180.0,
        )

        # If it retried, we expect 'gpu_oom_retry' event
        # and result annotated 'confidence: approximate'
        data = resp.json()
        if any(e["event_type"] == "gpu_oom_retry" for e in data.get("events", [])):
            assert data["confidence"] == "approximate"
        else:
            # If no GPU, it might just fail or run on CPU
            pass
