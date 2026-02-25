import asyncio
import os
import time
from pathlib import Path

import httpx
import pytest
import yaml

from controller.api.schemas import AgentRunResponse, EpisodeResponse
from controller.api.tasks import AgentRunRequest
from shared.enums import EpisodeStatus
from shared.models.schemas import (
    BoundingBox,
    Constraints,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ExecuteRequest,
    ExecuteResponse,
    ReadFileRequest,
    VerificationRequest,
    WriteFileRequest,
)
from tests.integration.contracts import HealthResponse

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


async def get_bundle(client: httpx.AsyncClient, session_id: str) -> str:
    """Fetch gzipped workspace from light worker and return base64 string."""
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/bundle",
        headers={"X-Session-ID": session_id},
        timeout=120.0,
    )
    assert resp.status_code == 200, f"Failed to get bundle: {resp.text}"
    import base64

    return base64.b64encode(resp.content).decode("utf-8")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_001_compose_boot_health_contract():
    """INT-001: Verify services are up and healthy."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Worker Light health
        resp = await client.get(f"{WORKER_LIGHT_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        light_health = HealthResponse.model_validate(resp.json())
        assert light_health.status.value == "HEALTHY"

        # Worker Heavy health
        resp = await client.get(f"{WORKER_HEAVY_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        heavy_health = HealthResponse.model_validate(resp.json())
        assert heavy_health.status.value == "HEALTHY"

        # Controller health
        resp = await client.get(f"{CONTROLLER_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        controller_health = HealthResponse.model_validate(resp.json())
        assert controller_health.status.value == "HEALTHY"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_002_controller_worker_execution_boundary():
    """INT-002: Verify controller-worker handoff and execution status."""
    session_id = f"INT-002-{int(time.time())}"
    task = "Build a simple box of 10x10x10mm."

    async with httpx.AsyncClient(timeout=300.0) as client:
        # Trigger agent run
        req = AgentRunRequest(task=task, session_id=session_id)
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=req.model_dump(mode="json"),
            timeout=600.0,
        )
        assert resp.status_code == 202
        agent_run_resp = AgentRunResponse.model_validate(resp.json())
        episode_id = agent_run_resp.episode_id

        # Poll for status with increased timeout
        max_attempts = 120
        completed = False
        for _ in range(max_attempts):
            await asyncio.sleep(10.0)
            s_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            ep_data = EpisodeResponse.model_validate(s_resp.json())
            status = ep_data.status
            if status == EpisodeStatus.COMPLETED:
                completed = True
                break
            if status == EpisodeStatus.FAILED:
                pytest.fail(f"Agent failed: {s_resp.text}")

        assert completed, "Agent did not complete in time"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_003_session_filesystem_isolation(worker_light_client):
    """INT-003: Verify sessions have isolated filesystems."""
    client = worker_light_client
    session_a = f"test-iso-a-{int(time.time())}"
    session_b = f"test-iso-b-{int(time.time())}"

    # Write to A
    req_write = WriteFileRequest(path="test_iso.txt", content="dummy_payload")
    await client.post(
        "/fs/write",
        json=req_write.model_dump(mode="json"),
        headers={"X-Session-ID": session_a},
    )

    # Try read from B
    req_read = ReadFileRequest(path="test_iso.txt")
    resp = await client.post(
        "/fs/read",
        json=req_read.model_dump(mode="json"),
        headers={"X-Session-ID": session_b},
    )
    assert resp.status_code == 404


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_004_simulation_serialization():
    """INT-004: Verify simulation serialization schema and concurrency."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Create two sessions
        session_id_1 = f"test-ser-1-{int(time.time())}"
        session_id_2 = f"test-ser-2-{int(time.time())}"

        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    b.label = "target_box"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return b
"""
        # Write scripts
        req_write = WriteFileRequest(path="box.py", content=script)
        await asyncio.gather(
            client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=req_write.model_dump(mode="json"),
                headers={"X-Session-ID": session_id_1},
            ),
            client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json=req_write.model_dump(mode="json"),
                headers={"X-Session-ID": session_id_2},
            ),
        )

        # Launch two simulations concurrently
        bundle1 = await get_bundle(client, session_id_1)
        bundle2 = await get_bundle(client, session_id_2)

        sim_req = BenchmarkToolRequest(
            script_path="box.py",
            backend=SimulatorBackendType.GENESIS,
            bundle_base64=bundle1,
            smoke_test_mode=True,
        )
        sim_req2 = sim_req.model_copy(update={"bundle_base64": bundle2})

        res1, res2 = await asyncio.gather(
            client.post(
                f"{WORKER_HEAVY_URL}/benchmark/simulate",
                json=sim_req.model_dump(mode="json"),
                headers={"X-Session-ID": session_id_1},
                timeout=600.0,
            ),
            client.post(
                f"{WORKER_HEAVY_URL}/benchmark/simulate",
                json=sim_req2.model_dump(mode="json"),
                headers={"X-Session-ID": session_id_2},
                timeout=600.0,
            ),
        )
        # Verify both succeeded
        assert res1.status_code == 200, f"Sim 1 failed: {res1.text}"
        assert res2.status_code == 200, f"Sim 2 failed: {res2.text}"

        data1 = BenchmarkToolResponse.model_validate(res1.json())
        data2 = BenchmarkToolResponse.model_validate(res2.json())

        assert data1.success or "Simulation stable" in data1.message, (
            f"Sim 1 marked failure: {data1.message}"
        )
        assert data2.success or "Simulation stable" in data2.message, (
            f"Sim 2 marked failure: {data2.message}"
        )

        # Check artifacts
        assert data1.artifacts.mjcf_content is not None
        assert data2.artifacts.mjcf_content is not None


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_020_simulation_failure_taxonomy():
    """INT-020: Verify simulation success/failure taxonomy."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-020-{int(time.time())}"

        # 1. Setup minimal objectives.yaml
        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10.5, 10.5, 10.5), max=(12.5, 12.5, 12.5)),
                forbid_zones=[
                    {
                        "name": "zone_forbid_test",
                        "min": (2.5, 2.5, 2.5),
                        "max": (4.5, 4.5, 4.5),
                    }
                ],
                build_zone=BoundingBox(min=(0.5, 0.5, 0.5), max=(20.5, 20.5, 20.5)),
            ),
            simulation_bounds=BoundingBox(
                min=(-20.5, -20.5, -20.5), max=(20.5, 20.5, 20.5)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(3.5, 3.5, 3.5),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=20.5, max_weight_g=10.5),
        )

        req_write_obj = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_obj.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        script_fail = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    with BuildPart() as p:
        Box(2, 2, 2, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    p.part.move(Location((3.5, 3.5, 3.5)))
    p.part.label = "target_box"
    p.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return p.part
"""

        req_write_fail = WriteFileRequest(path="fail.py", content=script_fail)
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_fail.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # DEBUG: add manual debug step to try and export stl
        debug_stl_script = """
import os
import sys
from build123d import *
def run():
    part = Box(1,1,1)
    export_stl(part, "debug_test.stl")
    if not os.path.exists("debug_test.stl"):
        raise RuntimeError("Manual export_stl failed")
run()
"""
        req_write_debug = WriteFileRequest(
            path="debug_stl.py", content=debug_stl_script
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_debug.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        resp_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "import sys; sys.path.append('.'); "
                    "import debug_stl; debug_stl.run()"
                ),
                timeout=120,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=180.0,
        )
        assert resp_exec.status_code == 200
        exec_data = ExecuteResponse.model_validate(resp_exec.json())
        assert exec_data.exit_code == 0

        bundle64 = await get_bundle(client, session_id)

        sim_req = BenchmarkToolRequest(
            script_path="fail.py",
            backend=SimulatorBackendType.GENESIS,
            bundle_base64=bundle64,
            smoke_test_mode=True,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=600.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        assert not data.success
        msg_lower = data.message.lower()
        assert any(
            msg in msg_lower
            for msg in [
                "forbid zone hit",
                "collision_with_forbidden_zone",
                "forbid_zone_hit",
            ]
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_021_runtime_randomization_robustness():
    """INT-021: Verify runtime randomization robustness (multi-seed)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-021-{int(time.time())}"

        objectives = ObjectivesYaml(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                build_zone=BoundingBox(min=(-20, -20, -20), max=(20, 20, 20)),
            ),
            simulation_bounds=BoundingBox(min=(-20, -20, -20), max=(20, 20, 20)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0.1, 0.1, 0),
            ),
            constraints=Constraints(max_unit_cost=20.0, max_weight_g=10.0),
        )
        req_write_obj = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_obj.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(0.1, 0.1, 0.1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    b.label = "target_box"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return b
"""
        req_write_script = WriteFileRequest(path="script.py", content=script_content)
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        bundle64 = await get_bundle(client, session_id)

        verify_req = VerificationRequest(
            script_path="script.py",
            bundle_base64=bundle64,
            jitter_range=(0.002, 0.002, 0.001),
            num_runs=3,
            duration=1.0,
            seed=42,
            backend=SimulatorBackendType.MUJOCO,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/verify",
            json=verify_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        assert data.success
        assert data.artifacts.verification_result is not None
        ver_result = data.artifacts.verification_result
        assert ver_result.num_runs == 3
        assert ver_result.success_rate == 1.0


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_022_motor_overload_behavior(worker_light_client):
    """INT-022: Verify motor overload detection."""
    client = worker_light_client
    session_id = f"test-int-022-{int(time.time())}"

    script_path = Path("tests/integration/architecture_p0/scripts/verify_overload.py")
    with script_path.open() as f:
        script_content = f.read()

    await client.post(
        "/fs/write",
        json=WriteFileRequest(
            path="verify_overload.py", content=script_content
        ).model_dump(mode="json"),
        headers={"X-Session-ID": session_id},
    )

    resp = await client.post(
        "/runtime/execute",
        json=ExecuteRequest(
            code=(
                "import sys; sys.path.append('.'); import verify_overload; "
                "import asyncio; asyncio.run(verify_overload.run())"
            ),
            timeout=90,
        ).model_dump(mode="json"),
        headers={"X-Session-ID": session_id},
        timeout=120.0,
    )
    assert resp.status_code == 200
    data = ExecuteResponse.model_validate(resp.json())

    assert data.exit_code == 0


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_023_fastener_validity_rules():
    """INT-023: Verify fastener validity rules."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-023-{int(time.time())}"

        script_valid = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p1 = Box(10, 10, 10)
    p2 = Cylinder(2, 20).move(Location((0,0,0)))
    res = p1.cut(p2)
    res.label = "valid_part"
    res.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return res
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="valid_hole.py", content=script_valid
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        bundle64 = await get_bundle(client, session_id)

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="valid_hole.py", bundle_base64=bundle64
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=180.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        assert data.success
