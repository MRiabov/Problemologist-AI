import asyncio
import io
import os
import time
import uuid
from pathlib import Path

import httpx
import pytest
import yaml
from PIL import Image

from controller.api.schemas import AgentRunRequest, AgentRunResponse, EpisodeResponse
from shared.enums import EpisodeStatus
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
    ExecuteRequest,
    ExecuteResponse,
    ListFilesRequest,
    PreviewDesignRequest,
    PreviewDesignResponse,
    ReadFileRequest,
    VerificationRequest,
    WriteFileRequest,
)
from tests.integration.agent.helpers import (
    seed_benchmark_assembly_definition,
    seed_execution_reviewer_handover,
)
from tests.integration.backend_utils import selected_backend
from tests.integration.contracts import HealthResponse

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
WORKER_RENDERER_URL = os.getenv("WORKER_RENDERER_URL", "http://127.0.0.1:18003")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {
                "fixed": True,
                "material_id": "aluminum_6061",
            },
        }
    ]


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

        # Worker Renderer health
        resp = await client.get(f"{WORKER_RENDERER_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        renderer_health = HealthResponse.model_validate(resp.json())
        assert renderer_health.status.value == "HEALTHY"

        # Controller health
        resp = await client.get(f"{CONTROLLER_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        controller_health = HealthResponse.model_validate(resp.json())
        assert controller_health.status.value == "HEALTHY"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_208_renderer_worker_direct_preview_contract():
    """INT-208: Renderer worker serves preview requests directly."""
    script_content = """
from build123d import Align, Box
from shared.models.schemas import PartMetadata

part = Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
part.label = "renderer_worker_smoke_box"
part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
result = part
"""

    async with httpx.AsyncClient(timeout=300.0) as client:
        resp = await client.post(
            f"{WORKER_RENDERER_URL}/benchmark/preview",
            json=PreviewDesignRequest(
                script_path="script.py",
                script_content=script_content,
                pitch=-35.0,
                yaw=45.0,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": f"INT-208-{uuid.uuid4().hex[:8]}"},
            timeout=180.0,
        )
        assert resp.status_code == 200, resp.text
        preview = PreviewDesignResponse.model_validate(resp.json())
        assert preview.success, preview.message
        assert preview.image_path is not None
        assert preview.image_path.startswith("renders/"), preview
        assert preview.image_bytes_base64 is not None


def _simulation_video_smoke_script() -> str:
    return """
from build123d import Align, Box
from shared.models.schemas import PartMetadata

def build():
    part = Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    part.label = "simulation_video_smoke_box"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return part
"""


async def _assert_simulation_video_contract(
    *,
    client: httpx.AsyncClient,
    session_id: str,
    backend: SimulatorBackendType,
) -> None:
    await seed_benchmark_assembly_definition(client, session_id)
    await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path="script.py",
            content=_simulation_video_smoke_script(),
            overwrite=True,
        ).model_dump(mode="json"),
        headers={"X-Session-ID": session_id},
    )

    bundle64 = await get_bundle(client, session_id)
    resp = await client.post(
        f"{WORKER_HEAVY_URL}/benchmark/simulate",
        json=BenchmarkToolRequest(
            script_path="script.py",
            backend=backend,
            bundle_base64=bundle64,
            smoke_test_mode=True,
        ).model_dump(mode="json"),
        headers={"X-Session-ID": session_id},
        timeout=1200.0,
    )
    assert resp.status_code == 200, resp.text
    data = BenchmarkToolResponse.model_validate(resp.json())
    assert data.success, data.message
    assert data.artifacts is not None

    render_paths = list(data.artifacts.render_paths)
    assert any(path.endswith(".mp4") for path in render_paths), render_paths
    assert "renders/render_manifest.json" in data.artifacts.render_blobs_base64
    mp4_keys = [
        path for path in data.artifacts.render_blobs_base64 if path.endswith(".mp4")
    ]
    assert mp4_keys, data.artifacts.render_blobs_base64
    for key in mp4_keys:
        assert data.artifacts.render_blobs_base64[key], key


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_210_mujoco_simulation_video_delegates_to_renderer_worker():
    """INT-210: MuJoCo simulation video is encoded by worker-renderer."""
    if selected_backend() != SimulatorBackendType.MUJOCO:
        pytest.skip("MuJoCo video smoke only runs when MuJoCo is the selected backend")

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _assert_simulation_video_contract(
            client=client,
            session_id=f"INT-210-{uuid.uuid4().hex[:8]}",
            backend=SimulatorBackendType.MUJOCO,
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_211_genesis_simulation_video_delegates_to_renderer_worker():
    """INT-211: Genesis simulation video uses the same renderer-worker contract."""
    if selected_backend() != SimulatorBackendType.GENESIS:
        pytest.skip(
            "Genesis video smoke only runs when Genesis is the selected backend"
        )

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _assert_simulation_video_contract(
            client=client,
            session_id=f"INT-211-{uuid.uuid4().hex[:8]}",
            backend=SimulatorBackendType.GENESIS,
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_002_controller_worker_execution_boundary():
    """INT-002: Verify controller-worker handoff and execution status."""
    session_id = f"INT-002-{uuid.uuid4().hex[:8]}"
    task = "Build a simple box of 10x10x10mm."

    async with httpx.AsyncClient(timeout=300.0) as client:
        await seed_benchmark_assembly_definition(client, session_id)
        await seed_execution_reviewer_handover(
            client,
            session_id=session_id,
            int_id="INT-002",
        )

        # Trigger agent run
        req = AgentRunRequest(task=task, session_id=session_id)
        resp = await client.post(
            f"{CONTROLLER_URL}/api/agent/run",
            json=req.model_dump(mode="json"),
            timeout=1000.0,
        )
        assert resp.status_code == 202
        agent_run_resp = AgentRunResponse.model_validate(resp.json())
        episode_id = agent_run_resp.episode_id

        # Poll for status with increased timeout
        max_attempts = 120
        completed = False
        for _ in range(max_attempts):
            await asyncio.sleep(10.0)
            s_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
            ep_data = EpisodeResponse.model_validate(s_resp.json())
            status = ep_data.status
            if status == EpisodeStatus.COMPLETED:
                completed = True
                break
            if status == EpisodeStatus.FAILED:
                pytest.fail(f"Agent failed: {ep_data.model_dump(exclude={'traces'})}")

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
    """INT-004: heavy worker exposes single-flight admission, busy responses, and readiness gating."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_ids = [f"test-ser-{i}-{int(time.time())}" for i in range(3)]

        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    b.label = "target_box"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return b
"""
        req_write = WriteFileRequest(path="box.py", content=script)
        await asyncio.gather(
            *[
                client.post(
                    f"{WORKER_LIGHT_URL}/fs/write",
                    json=req_write.model_dump(mode="json"),
                    headers={"X-Session-ID": sid},
                )
                for sid in session_ids
            ]
        )

        bundles = [await get_bundle(client, sid) for sid in session_ids]

        # Avoid cross-test overlap by waiting for an idle heavy worker before INT-004 starts.
        for _ in range(120):
            ready_resp = await client.get(f"{WORKER_HEAVY_URL}/ready", timeout=5.0)
            if ready_resp.status_code == 200:
                break
            await asyncio.sleep(0.5)
        else:
            pytest.fail("worker-heavy did not become ready before INT-004")

        async def _simulate(session_id: str, bundle_base64: str):
            sim_req = BenchmarkToolRequest(
                script_path="box.py",
                backend=selected_backend(),
                bundle_base64=bundle_base64,
                smoke_test_mode=True,
            )
            resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/simulate",
                json=sim_req.model_dump(mode="json"),
                headers={"X-Session-ID": session_id},
                timeout=1000.0,
            )
            try:
                payload = resp.json()
            except Exception:
                payload = {"raw_text": resp.text}
            return resp.status_code, payload

        first_task = asyncio.create_task(_simulate(session_ids[0], bundles[0]))
        observed_not_ready = False
        for _ in range(80):
            if first_task.done():
                break
            ready_resp = await client.get(f"{WORKER_HEAVY_URL}/ready", timeout=5.0)
            if ready_resp.status_code != 200:
                observed_not_ready = True
                break
            await asyncio.sleep(0.1)

        assert observed_not_ready, (
            "expected /ready to report not-ready while the admitted heavy job was active"
        )

        remaining_results = await asyncio.gather(
            *[
                _simulate(sid, bundle)
                for sid, bundle in zip(session_ids[1:], bundles[1:])
            ]
        )
        results = [await first_task, *remaining_results]

        admitted_successes = 0
        busy_count = 0
        unexpected: list[str] = []
        for idx, (status_code, payload) in enumerate(results):
            if status_code == 200:
                parsed = BenchmarkToolResponse.model_validate(payload)
                if parsed.success:
                    admitted_successes += 1
                else:
                    unexpected.append(
                        f"request[{idx}] returned 200 but success=false: {parsed.message}"
                    )
                continue

            if status_code == 503:
                detail = payload.get("detail") if isinstance(payload, dict) else None
                if isinstance(detail, dict) and detail.get("code") == "WORKER_BUSY":
                    busy_count += 1
                else:
                    unexpected.append(
                        f"request[{idx}] returned 503 without WORKER_BUSY: {payload}"
                    )
                continue

            unexpected.append(
                f"request[{idx}] unexpected status={status_code}: {payload}"
            )

        assert not unexpected, "\n".join(unexpected)
        assert admitted_successes == 1, (
            f"expected exactly 1 admitted request, got {admitted_successes}"
        )
        assert busy_count >= 1, "expected one or more WORKER_BUSY responses"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_020_simulation_failure_taxonomy():
    """INT-020: Verify simulation success/failure taxonomy."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-020-{int(time.time())}"

        # 1. Setup minimal benchmark_definition.yaml
        objectives = BenchmarkDefinition(
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
            benchmark_parts=_default_benchmark_parts(),
            simulation_bounds=BoundingBox(
                min=(-20.5, -20.5, -20.5), max=(20.5, 20.5, 20.5)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.5, 0.5, 0.5),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=20.5, max_weight_g=10.5),
        )

        req_write_obj = WriteFileRequest(
            path="benchmark_definition.yaml",
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
                code=("python - <<'PY'\nimport debug_stl\ndebug_stl.run()\nPY\n"),
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
            backend=selected_backend(),
            bundle_base64=bundle64,
            smoke_test_mode=True,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=1000.0,
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

        # Success path for taxonomy: explicit goal completion signal.
        success_objectives = objectives.model_copy(deep=True)
        success_objectives.objectives.forbid_zones = []
        success_objectives.objectives.goal_zone = BoundingBox(
            min=(3.0, 3.0, 3.0), max=(4.0, 4.0, 4.0)
        )
        req_write_success_obj = WriteFileRequest(
            path="benchmark_definition.yaml",
            content=yaml.dump(success_objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_success_obj.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        success_bundle64 = await get_bundle(client, session_id)
        success_sim_req = sim_req.model_copy(update={"bundle_base64": success_bundle64})
        success_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=success_sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=1000.0,
        )
        assert success_resp.status_code == 200
        success_data = BenchmarkToolResponse.model_validate(success_resp.json())
        if not success_data.success:
            pytest.fail(f"Success path failed: {success_data.message}")
        assert (
            "goal achieved" in success_data.message.lower()
            or "goal zone" in success_data.message.lower()
            or "green zone" in success_data.message.lower()
        ), success_data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_021_runtime_randomization_robustness():
    """INT-021: Verify runtime randomization robustness (multi-seed)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-021-{int(time.time())}"

        await seed_benchmark_assembly_definition(client, session_id)

        objectives = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                build_zone=BoundingBox(min=(-20, -20, -20), max=(20, 20, 20)),
            ),
            benchmark_parts=_default_benchmark_parts(),
            simulation_bounds=BoundingBox(min=(-20, -20, -20), max=(20, 20, 20)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0.1, 0.1, 0),
            ),
            constraints=Constraints(max_unit_cost=20.0, max_weight_g=10.0),
        )
        req_write_obj = WriteFileRequest(
            path="benchmark_definition.yaml",
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
    with BuildPart() as p:
        Box(0.1, 0.1, 0.1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    p.part.label = "target_box"
    p.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return p.part
"""
        req_write_script = WriteFileRequest(path="script.py", content=script_content)
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="script.py",
                backend=selected_backend(),
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message
        assert validate_data.artifacts is not None
        assert validate_data.artifacts.validation_results_json is not None

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="validation_results.json",
                content=validate_data.artifacts.validation_results_json,
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        verify_req = VerificationRequest(
            script_path="script.py",
            jitter_range=(0.002, 0.002, 0.001),
            num_scenes=3,
            duration=1.0,
            seed=42,
            backend=selected_backend(),
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/verify",
            json=verify_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        if not data.success:
            pytest.fail(f"Verification failed: {data.message}")
        assert data.artifacts.verification_result is not None
        ver_result = data.artifacts.verification_result
        assert ver_result.num_scenes == 3
        assert ver_result.success_rate == 1.0
        assert ver_result.scene_build_count == 1
        assert ver_result.backend_run_count == 1
        assert ver_result.batched_execution is True


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
                "python - <<'PY'\n"
                "import asyncio\n"
                "import verify_overload\n"
                "asyncio.run(verify_overload.run())\n"
                "PY\n"
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

        if not data.success:
            pytest.fail(f"Fastener validation failed: {data.message}")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_024_worker_benchmark_validation_toolchain():
    """
    INT-024: /benchmark/validate fails on invalid objective setups,
    including jitter-range conflicts.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-024-{uuid.uuid4().hex[:8]}"

        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(path="script.py", content=script).model_dump(
                mode="json"
            ),
            headers={"X-Session-ID": session_id},
        )

        overlap_objectives = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(0.0, 0.0, 0.0), max=(10.0, 10.0, 10.0)),
                forbid_zones=[
                    {
                        "name": "goal_overlap",
                        "min": (5.0, 5.0, 5.0),
                        "max": (12.0, 12.0, 12.0),
                    }
                ],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            benchmark_parts=_default_benchmark_parts(),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="benchmark_definition.yaml",
                content=yaml.dump(overlap_objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        overlap_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=180.0,
        )
        assert overlap_resp.status_code == 200
        overlap_data = BenchmarkToolResponse.model_validate(overlap_resp.json())
        assert overlap_data.success is False
        assert "goal_zone overlaps forbid zone" in (overlap_data.message or "")

        jitter_conflict_objectives = overlap_objectives.model_copy(
            update={
                "objectives": ObjectivesSection(
                    goal_zone=BoundingBox(
                        min=(12.0, 12.0, 0.0),
                        max=(16.0, 16.0, 6.0),
                    ),
                    forbid_zones=[
                        {
                            "name": "spawn_conflict",
                            "min": (-3.0, -3.0, 0.0),
                            "max": (3.0, 3.0, 6.0),
                        }
                    ],
                    build_zone=BoundingBox(
                        min=(-20.0, -20.0, 0.0),
                        max=(20.0, 20.0, 30.0),
                    ),
                ),
                "moved_object": MovedObject(
                    label="target_box",
                    shape="sphere",
                    material_id="aluminum_6061",
                    start_position=(0.0, 0.0, 3.0),
                    runtime_jitter=(2.0, 2.0, 1.0),
                ),
            }
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="benchmark_definition.yaml",
                content=yaml.dump(jitter_conflict_objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        jitter_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=180.0,
        )
        assert jitter_resp.status_code == 200
        jitter_data = BenchmarkToolResponse.model_validate(jitter_resp.json())
        assert jitter_data.success is False
        assert "moved_object runtime envelope intersects forbid zone" in (
            jitter_data.message or ""
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_188_validation_preview_uses_build123d_even_for_genesis_objectives():
    """
    INT-188: /benchmark/validate routes static preview rendering to build123d/VTK
    even when objectives request Genesis for physics simulation.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-188-{uuid.uuid4().hex[:8]}"
        headers = {"X-Session-ID": session_id}

        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(10, 10, 10).move(Location((0, 0, 5)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
        write_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py", content=script, overwrite=True
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert write_resp.status_code == 200, write_resp.text

        objectives = BenchmarkDefinition(
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(12.0, 12.0, 0.0), max=(16.0, 16.0, 6.0)),
                forbid_zones=[],
                build_zone=BoundingBox(min=(-20.0, -20.0, 0.0), max=(20.0, 20.0, 30.0)),
            ),
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            benchmark_parts=_default_benchmark_parts(),
            simulation_bounds=BoundingBox(
                min=(-50.0, -50.0, -10.0), max=(50.0, 50.0, 50.0)
            ),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0.0, 0.0, 10.0),
                runtime_jitter=(0.0, 0.0, 0.0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=1000.0),
        )
        objectives_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="benchmark_definition.yaml",
                content=yaml.dump(objectives.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert objectives_resp.status_code == 200, objectives_resp.text

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers=headers,
            timeout=180.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_payload = validate_resp.json()
        validate_data = BenchmarkToolResponse.model_validate(validate_payload)
        assert validate_data.success, validate_data.message
        assert validate_data.artifacts is not None
        assert validate_data.artifacts.validation_results_json is not None

        benchmark_render_events = [
            event
            for event in validate_payload.get("events", [])
            if event.get("event_type") == "render_request_benchmark"
        ]
        assert benchmark_render_events, validate_payload.get("events", [])
        assert any(
            event.get("backend") == "build123d_vtk"
            and event.get("purpose") == "validation_static_preview"
            for event in benchmark_render_events
        ), benchmark_render_events

        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="renders").model_dump(mode="json"),
            headers=headers,
        )
        assert ls_resp.status_code == 200, ls_resp.text
        render_entries = ls_resp.json()
        render_names = [
            entry["name"] for entry in render_entries if not entry["is_dir"]
        ]
        png_renders = [name for name in render_names if name.endswith(".png")]
        assert png_renders, render_names

        blob_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read_blob",
            json=ReadFileRequest(path=f"renders/{png_renders[0]}").model_dump(
                mode="json"
            ),
            headers=headers,
        )
        assert blob_resp.status_code == 200, blob_resp.text
        image = Image.open(io.BytesIO(blob_resp.content)).convert("RGB")
        extrema = image.getextrema()
        assert any(high > 0 for _, high in extrema), extrema
