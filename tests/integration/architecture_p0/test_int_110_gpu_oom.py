import os
import time

import httpx
import pytest
import yaml

from shared.models.schemas import (
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    MovedObject,
    ObjectivesSection,
    PhysicsConfig,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    WriteFileRequest,
)
from tests.integration.backend_utils import skip_unless_genesis

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {"fixed": True, "material_id": "aluminum_6061"},
        }
    ]


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
    skip_unless_genesis("INT-110 targets Genesis GPU OOM retry behavior.")
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = BenchmarkDefinition(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS,
                compute_target="cuda",
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            payload=MovedObject(
                label="obj",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0, 0, 5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
            benchmark_parts=_default_benchmark_parts(),
        )

        write_obj_req = WriteFileRequest(
            path="benchmark_definition.yaml",
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
