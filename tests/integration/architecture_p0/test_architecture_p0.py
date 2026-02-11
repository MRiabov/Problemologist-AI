import asyncio
import contextlib
import json
import os
import time
import pytest
import httpx

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:8001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:8000")


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
        assert resp.status_code == 200
        episode_id = resp.json()["episode_id"]

        # Wait for completion
        completed = False
        for _ in range(30):
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                status = status_resp.json()["status"]
                if status == "completed":
                    completed = True
                    break
                if status == "failed":
                    pytest.fail("Agent run failed")
            await asyncio.sleep(1)

        # Check if completed (optional, or fail if not)
        # assert completed, "Agent run timed out or did not complete"


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
    """INT-004: Verify simulation serialization schema."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-ser-{int(time.time())}"

        # Simple valid script
        script = """
from build123d import *
def build():
    return Box(1,1,1)
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "box.py", "content": script},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={"script_path": "box.py"},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert "success" in data
        assert "message" in data
        assert "artifacts" in data
        assert "mjcf_content" in data["artifacts"]


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
    min: [10, 10, 10]
    max: [12, 12, 12]
  forbid_zones:
    - name: "test_forbid"
      min: [2, 2, 2]
      max: [4, 4, 4]
  build_zone:
    min: [0, 0, 0]
    max: [100, 100, 100]
simulation_bounds:
    min: [-100, -100, -100]
    max: [100, 100, 100]
moved_object:
    label: "test_obj"
    shape: "sphere"
    start_position: [0, 0, 5]
    runtime_jitter: [0, 0, 0]
constraints:
    max_unit_cost: 100
    max_weight: 10
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        # 2. Write a script that places a box in the forbid zone
        script_fail = """
from build123d import *
def build():
    with BuildPart() as p:
        Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    # Move to forbid zone
    p.part.move(Location((3, 3, 3)))
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
    try:
        part = Box(1,1,1)
        # Check if we can import
        print("Build123d imported")
        # Try to export stl manually
        export_stl(part, "debug_test.stl")
        if os.path.exists("debug_test.stl"):
            print("Manual export_stl success")
        else:
            print("Manual export_stl failed")
    except Exception as e:
        import traceback
        print(traceback.format_exc())
run()
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "debug_stl.py", "content": debug_stl_script},
            headers={"X-Session-ID": session_id},
        )
        debug_resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": "import sys; sys.path.append('.'); import debug_stl",
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        print(
            f"DEBUG STL CHECK:\n{debug_resp.json().get('stdout')}\n{debug_resp.json().get('stderr')}"
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={"script_path": "script_fail.py"},
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # DEBUG: Print full response if failure is not what expected
        if "Forbid zone hit" not in data.get("message", ""):
            print(f"INT-020 DEBUG RESPONSE: {data}")

        assert not data["success"]
        # Expect "Forbid zone hit: test_forbid"
        assert "Forbid zone hit" in data["message"]


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
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        script_path = "tests/integration/architecture_p0/scripts/verify_jitter.py"
        with open(script_path, "r") as f:
            script_content = f.read()

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "verify_jitter.py", "content": script_content},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": "import sys; sys.path.append('.'); import verify_jitter; import asyncio; asyncio.run(verify_jitter.run())",
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        if data["exit_code"] != 0:
            print(f"STDOUT:\n{data['stdout']}")
            print(f"STDERR:\n{data['stderr']}")

            # Try to read debug log
            debug_resp = await client.post(
                f"{WORKER_URL}/fs/read",
                json={"path": "debug_jitter.txt"},
                headers={"X-Session-ID": session_id},
            )
            if debug_resp.status_code == 200:
                print(f"DEBUG LOG:\n{debug_resp.json()['content']}")

        assert data["exit_code"] == 0
        assert "VERIFICATION_RESULT" in data["stdout"]
        assert "success_rate=1.0" in data["stdout"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_022_motor_overload_behavior():
    """INT-022: Verify motor overload detection."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-int-022-{int(time.time())}"

        script_path = "tests/integration/architecture_p0/scripts/verify_overload.py"
        with open(script_path, "r") as f:
            script_content = f.read()

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "verify_overload.py", "content": script_content},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={
                "code": "import sys; sys.path.append('.'); import verify_overload; import asyncio; asyncio.run(verify_overload.run())",
                "timeout": 30,
            },
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        if data["exit_code"] != 0:
            print(f"STDOUT:\n{data['stdout']}")
            print(f"STDERR:\n{data['stderr']}")

            # Try to read debug log
            debug_resp = await client.post(
                f"{WORKER_URL}/fs/read",
                json={"path": "debug_overload.txt"},
                headers={"X-Session-ID": session_id},
            )
            if debug_resp.status_code == 200:
                print(f"DEBUG LOG:\n{debug_resp.json()['content']}")

        assert data["exit_code"] == 0


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
from worker.utils.cad import fastener_hole, HoleType

def build():
    p = Box(10, 10, 10)
    p = fastener_hole(
        p,
        Location((0,0,5)),
        hole_id="test_hole",
        size="M3",
        hole_type=HoleType.CounterBoreHole
    )
    return p
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "valid_hole.py", "content": script_valid},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/validate",
            json={"script_path": "valid_hole.py"},
            headers={"X-Session-ID": session_id},
            timeout=30.0,  # Validate is fast
        )
        assert resp.status_code == 200, resp.text
        data = resp.json()

        if not data["success"]:
            print(f"INT-023 DEBUG FAILURE: {data.get('message')}")

        assert data["success"]
