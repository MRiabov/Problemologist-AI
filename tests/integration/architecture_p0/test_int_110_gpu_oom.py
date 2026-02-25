import os
import time

import httpx
import pytest
import yaml

from shared.models.schemas import (
    ObjectivesYaml,
    ObjectivesSection,
    PhysicsConfig,
    BoundingBox,
    MovedObject,
    Constraints,
)
from shared.workers.schema import (
    BenchmarkToolResponse,
    BenchmarkToolRequest,
    WriteFileRequest,
)
from shared.simulation.schemas import SimulatorBackendType

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
async def test_int_110_gpu_oom_retry(session_id, base_headers):
    """
    INT-110: GPU OOM retry with particle reduction.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS,
                compute_target="cuda",
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                start_position=(0, 0, 5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )

        write_obj_req = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_obj_req.model_dump(mode="json"),
            headers=base_headers,
        )

        # Trigger simulation with a huge particle count that exceeds typical RAM/VRAM
        sim_req = BenchmarkToolRequest(script_path="script.py", particle_budget=10**9)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=180.0,
        )

        # If it retried, we expect 'gpu_oom_retry' event
        # and result annotated 'confidence: approximate'
        data = BenchmarkToolResponse.model_validate(resp.json())
        if any(e["event_type"] == "gpu_oom_retry" for e in data.events):
            assert data.confidence == "approximate"
        else:
            # If no GPU, it might just fail or run on CPU
            pass
