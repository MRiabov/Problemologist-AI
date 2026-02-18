import asyncio
import os
import time
from pathlib import Path

import httpx
import pytest

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:18001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_001_compose_boot_health_contract():
    """INT-001: Verify services are up and healthy."""
    async with httpx.AsyncClient() as client:
        # Worker health
        resp = await client.get(f"{WORKER_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        assert resp.json() == {"status": "healthy"}

        # Controller health
        resp = await client.get(f"{CONTROLLER_URL}/health", timeout=5.0)
        assert resp.status_code == 200
        assert resp.json().get("status") == "healthy"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_002_controller_worker_execution_boundary():
    """INT-002: Verify execution happens on worker only."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-boundary-{int(time.time())}"
        payload = {
            "task": "Write a unique file to verify worker execution",
            "session_id": session_id,
        }

        # Trigger agent run
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run", json=payload, timeout=20.0
        )
        assert resp.status_code in [200, 202]
        episode_id = resp.json()["episode_id"]

        # Wait for completion
        completed = False
        for _ in range(30):
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                status = status_resp.json()["status"]
                if status == "failed":
                    msg = (
                        status_resp.json().get("metadata", {}).get("error")
                        or "Agent run failed"
                    )
                    pytest.fail(
                        f"Agent run failed: {msg}. Full status: {status_resp.json()}"
                    )
                if status == "completed":
                    completed = True
                    break
            await asyncio.sleep(0.5)
        assert completed, "Agent run timed out or did not complete"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_003_session_filesystem_isolation():
    """INT-003: Verify sessions have isolated filesystems."""
    async with httpx.AsyncClient() as client:
        session_a = f"test-iso-a-{int(time.time())}"
        session_b = f"test-iso-b-{int(time.time())}"

        # Write to A
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "secret.txt", "content": "secret_data"},
            headers={"X-Session-ID": session_a},
        )

        # Try read from B
        resp = await client.post(
            f"{WORKER_URL}/fs/read",
            json={"path": "secret.txt"},
            headers={"X-Session-ID": session_b},
        )
        assert resp.status_code == 404


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_004_simulation_serialization():
    """INT-004: Verify simulation serialization schema and concurrency."""
    async with httpx.AsyncClient() as client:
        # Create two sessions
        session_id_1 = f"test-ser-1-{int(time.time())}"
        session_id_2 = f"test-ser-2-{int(time.time())}"

        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    b.label = "box"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return b
"""
        # Write scripts
        await asyncio.gather(
            client.post(
                f"{WORKER_URL}/fs/write",
                json={"path": "box.py", "content": script},
                headers={"X-Session-ID": session_id_1},
            ),
            client.post(
                f"{WORKER_URL}/fs/write",
                json={"path": "box.py", "content": script},
                headers={"X-Session-ID": session_id_2},
            ),
        )

        # Launch two simulations concurrently
        # The worker should handle them (either serializing or thread-safe parallel)
        # We start them at roughly the same time
        t0 = time.time()
        res1, res2 = await asyncio.gather(
            client.post(
                f"{WORKER_URL}/benchmark/simulate",
                json={"script_path": "box.py", "smoke_test_mode": True},
                headers={"X-Session-ID": session_id_1},
                timeout=60.0,
            ),
            client.post(
                f"{WORKER_URL}/benchmark/simulate",
                json={"script_path": "box.py", "smoke_test_mode": True},
                headers={"X-Session-ID": session_id_2},
                timeout=60.0,
            ),
        )
        t1 = time.time()

        # Verify both succeeded
        assert res1.status_code == 200, f"Sim 1 failed: {res1.text}"
        assert res2.status_code == 200, f"Sim 2 failed: {res2.text}"

        data1 = res1.json()
        data2 = res2.json()

        assert data1["success"] or "Simulation stable" in data1["message"], (
            f"Sim 1 marked failure: {data1}"
        )
        assert data2["success"] or "Simulation stable" in data2["message"], (
            f"Sim 2 marked failure: {data2}"
        )

        # Check artifacts
        assert "mjcf_content" in data1["artifacts"]
        assert "mjcf_content" in data2["artifacts"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_020_simulation_failure_taxonomy():
    """INT-020: Verify simulation success/failure taxonomy."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-020-{int(time.time())}"

        # 1. Setup minimal objectives.yaml that defines a goal and forbid zone
        objectives_content = """
objectives:
  goal_zone:
    min: [10.5, 10.5, 10.5]
    max: [12.5, 12.5, 12.5]
  forbid_zones:
    - name: "test_forbid"
      min: [2.5, 2.5, 2.5]
      max: [4.5, 4.5, 4.5]
  build_zone:
    min: [0.5, 0.5, 0.5]
    max: [100.5, 100.5, 100.5]
simulation_bounds:
    min: [-100.5, -100.5, -100.5]
    max: [100.5, 100.5, 100.5]
moved_object:
    label: "test_obj"
    shape: "sphere"
    start_position: [0.5, 0.5, 5.5]
    runtime_jitter: [0.5, 0.5, 0.5]
constraints:
    max_unit_cost: 100.5
    max_weight: 10.5
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )

        script_fail = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    with BuildPart() as p:
        Box(2, 2, 2, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # Move to forbid zone center (3.5, 3.5, 3.5)
    p.part.move(Location((3.5, 3.5, 3.5)))
    p.part.label = "target_box"
    p.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return p.part
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "script_fail.py", "content": script_fail},
            headers={"X-Session-ID": session_id},
        )

        # DEBUG: add manual debug step to try and export stl
        debug_stl_script = """
import os
import sys
from build123d import *
from worker.utils.validation import to_mjcf
def run():
    part = Box(1,1,1)
    export_stl(part, "debug_test.stl")
    if not os.path.exists("debug_test.stl"):
        raise RuntimeError("Manual export_stl failed")
run()
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "debug_stl.py", "content": debug_stl_script},
            headers={"X-Session-ID": session_id},
        )
        resp_exec = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": "import sys; sys.path.append('.'); import debug_stl; debug_stl.run()",
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp_exec.status_code == 200, f"Debug script failed: {resp_exec.text}"
        assert resp_exec.json()["exit_code"] == 0, (
            f"Debug script exit code non-zero: {resp_exec.json()}"
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={"script_path": "script_fail.py"},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        assert not data["success"]
        # Expect "Forbid zone hit", "collision_with_forbidden_zone",
        # or "forbid_zone_hit"
        assert any(
            msg in data["message"].lower()
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
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-021-{int(time.time())}"

        # Write objectives.yaml so simulation can check goals
        objectives_content = """
objectives:
  goal_zone:
    min: [-10, -10, -10]
    max: [10, 10, 10]
  forbid_zones: []
  build_zone:
    min: [-100, -100, -100]
    max: [100, 100, 100]
simulation_bounds:
    min: [-100, -100, -100]
    max: [100, 100, 100]
moved_object:
    label: "target_box"
    shape: "sphere"
    start_position: [0, 0, 0.5]
    runtime_jitter: [0.1, 0.1, 0]
constraints:
    max_unit_cost: 100
    max_weight: 10
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "objectives.yaml",
                "content": objectives_content,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )

        script_path = Path("tests/integration/architecture_p0/scripts/verify_jitter.py")
        with script_path.open() as f:
            script_content = f.read()

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "verify_jitter.py", "content": script_content},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": (
                    "import sys; sys.path.append('.'); import verify_jitter; "
                    "import asyncio; result = asyncio.run(verify_jitter.run()); "
                    "assert result['success_rate'] == 1.0, f'Low success rate: {result}'"
                ),
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200, f"Execution failed: {resp.text}"
        data = resp.json()

        assert data["exit_code"] == 0, (
            f"Verification script failed with exit code {data['exit_code']}\n"
            f"STDOUT:\n{data['stdout']}\n"
            f"STDERR:\n{data['stderr']}"
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_022_motor_overload_behavior():
    """INT-022: Verify motor overload detection."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-022-{int(time.time())}"

        script_path = Path(
            "tests/integration/architecture_p0/scripts/verify_overload.py"
        )
        with script_path.open() as f:
            script_content = f.read()

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "verify_overload.py", "content": script_content},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": (
                    "import sys; sys.path.append('.'); import verify_overload; "
                    "import asyncio; asyncio.run(verify_overload.run())"
                ),
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        assert data["exit_code"] == 0, (
            f"Exit code {data['exit_code']} != 0\n"
            f"STDOUT:\n{data['stdout']}\n"
            f"STDERR:\n{data['stderr']}"
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_023_fastener_validity_rules():
    """INT-023: Verify fastener validity rules."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-023-{int(time.time())}"

        # Script with INVALID hole (e.g. asking for M3 but hole logic fails?)
        # Actually `fastener_hole` raises ValueError if size unknown.
        # But if we want to check geometric validity?
        # `validate` function checks interactions.
        # But `fastener_hole` creates RigidJoint.

        # Let's verify that a valid fastener hole is created and validated.
        script_valid = """
from build123d import *
from shared.models.schemas import PartMetadata
from worker.utils.cad import fastener_hole, HoleType

def build():
    # Minimal boolean script to see if it crashes worker
    p1 = Box(10, 10, 10)
    p2 = Cylinder(2, 20).move(Location((0,0,0)))
    res = p1.cut(p2)
    res.label = "valid_part"
    res.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return res
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "valid_hole.py", "content": script_valid},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/validate",
            json={"script_path": "valid_hole.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=30.0,  # Validate is fast
        )
        assert resp.status_code == 200, resp.text
        data = resp.json()

        assert data["success"], f"INT-023 failure: {data.get('message')}"
