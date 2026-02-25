import os
import uuid

import httpx
import pytest

from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    FsFileEntry,
    ListFilesRequest,
    SimulationArtifacts,
    WriteFileRequest,
)

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_138_smoke_test_mode():
    """INT-138: Verify smoke-test mode for Genesis."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-138-{uuid.uuid4().hex[:8]}"

        objectives_content = f"""
physics:
  backend: "{SimulatorBackendType.GENESIS}"
objectives:
  goal_zone: {{min: [10,10,10], max: [12,12,12]}}
  build_zone: {{min: [-100,-100,-100], max: [100,100,100]}}
simulation_bounds: {{min: [-100,-100,-100], max: [100,100,100]}}
moved_object: {{label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}}
constraints: {{max_unit_cost: 100, max_weight_g: 10}}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=objectives_content,
                overwrite=True,
            ).model_dump(),
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
            json=WriteFileRequest(
                path="script.py",
                content=script_content,
            ).model_dump(),
            headers={"X-Session-ID": session_id},
        )

        # Trigger simulation with smoke_test_mode=True
        request = BenchmarkToolRequest(
            script_path="script.py",
            smoke_test_mode=True,
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=request.model_dump(),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        # Verify result labelled approximate
        assert data.confidence == "approximate", (
            f"Expected confidence 'approximate', got '{data.confidence}'"
        )


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_139_fluid_storage_policy():
    """INT-139: Verify fluid data storage policy."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-139-{uuid.uuid4().hex[:8]}"

        objectives_content = f"""
physics:
  backend: "{SimulatorBackendType.GENESIS}"
objectives:
  goal_zone: {{min: [10,10,10], max: [12,12,12]}}
  build_zone: {{min: [-100,-100,-100], max: [100,100,100]}}
simulation_bounds: {{min: [-100,-100,-100], max: [100,100,100]}}
moved_object: {{label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}}
constraints: {{max_unit_cost: 100, max_weight_g: 10}}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="objectives.yaml",
                content=objectives_content,
                overwrite=True,
            ).model_dump(),
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
            json=WriteFileRequest(
                path="script.py",
                content=script_content,
            ).model_dump(),
            headers={"X-Session-ID": session_id},
        )

        request = BenchmarkToolRequest(
            script_path="script.py",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=request.model_dump(),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        # Verify artifacts in response (MP4, JSON, etc.)
        assert data.success, f"Simulation failed: {data.message}"
        artifacts = data.artifacts
        assert artifacts is not None, (
            f"Artifacts missing in response. Message: {data.message}"
        )

        if isinstance(artifacts, dict):
            artifacts = SimulationArtifacts.model_validate(artifacts)

        assert artifacts.render_paths is not None, (
            "render_paths is missing or None in artifacts"
        )

        # Verify raw particle data absent from workspace
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="/").model_dump(),
            headers={"X-Session-ID": session_id},
        )
        assert ls_resp.status_code == 200
        files = [FsFileEntry.model_validate(f) for f in ls_resp.json()]
        assert not any("particles" in f.name.lower() for f in files), (
            "Raw particle data found in workspace"
        )
