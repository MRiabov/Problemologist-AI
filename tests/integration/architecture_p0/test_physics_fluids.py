import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_101_physics_backend_selection():
    """INT-101: Verify physics backend selection and event emission."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-101-{int(time.time())}"

        # 1. Setup objectives.yaml with Genesis backend
        objectives_content = """
physics:
  backend: "genesis"
  fem_enabled: true
  compute_target: "cpu"
objectives:
  goal_zone:
    min: [10, 10, 10]
    max: [12, 12, 12]
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
    max_weight_g: 10
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        # 2. Simple box script
        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "script.py", "content": script},
            headers={"X-Session-ID": session_id},
        )

        # 3. Simulate
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        events = data.get("events", [])

        # Verify event emission
        event_types = [e.get("event_type") for e in events]
        assert "simulation_backend_selected" in event_types, (
            "Missing simulation_backend_selected event"
        )

        event = next(
            e for e in events if e["event_type"] == "simulation_backend_selected"
        )
        assert event["backend"] == "genesis"
        assert event["fem_enabled"] is True
        assert event["compute_target"] == "cpu"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_105_fluid_containment_evaluation():
    """INT-105: Verify fluid containment objective evaluation."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-105-{int(time.time())}"

        # Setup objectives with fluid containment
        # Zone contains (0,0,0)
        objectives_content = """
physics:
  backend: "genesis"
objectives:
  goal_zone: {min: [10,10,10], max: [12,12,12]}
  build_zone: {min: [-100,-100,-100], max: [100,100,100]}
  fluid_objectives:
    - type: "fluid_containment"
      fluid_id: "water"
      containment_zone:
        min: [-5, -5, -5]
        max: [5, 5, 5]
      threshold: 0.95
      eval_at: "end"
simulation_bounds: {min: [-100,-100,-100], max: [100,100,100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "script.py",
                "content": """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # Verify fluid metrics in result
        fluid_metrics = data.get("artifacts", {}).get("fluid_metrics", [])
        assert len(fluid_metrics) > 0
        metric = fluid_metrics[0]
        assert metric["metric_type"] == "fluid_containment"
        assert metric["fluid_id"] == "water"
        # Since dummy particles are at (0,0,0) and zone is [-5, 5], ratio should be 1.0
        assert metric["measured_value"] == 1.0
        assert metric["passed"] is True

        # 4. Fail path: threshold > 1.0 (impossible to achieve)
        objectives_fail = objectives_content.replace(
            "threshold: 0.95", "threshold: 1.1"
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_fail},
            headers={"X-Session-ID": session_id},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert not data["success"]
        assert "FLUID_OBJECTIVE_FAILED" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_106_flow_rate_evaluation():
    """INT-106: Verify flow rate objective evaluation (stub)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-106-{int(time.time())}"

        objectives_content = """
physics:
  backend: "genesis"
objectives:
  goal_zone: {min: [10,10,10], max: [12,12,12]}
  build_zone: {min: [-100,-100,-100], max: [100,100,100]}
  fluid_objectives:
    - type: "flow_rate"
      fluid_id: "water"
      gate_plane_point: [0, 0, 0]
      gate_plane_normal: [0, 0, 1]
      target_rate_l_per_s: 1.0
simulation_bounds: {min: [-100,-100,-100], max: [100,100,100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "script.py",
                "content": """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        # Flow rate evaluation is currently a no-op in SimulationLoop if not 'continuous'
        # but we check if it handles it without crashing.


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_112_mujoco_backward_compat():
    """INT-112: Verify MuJoCo ignores fluid/FEM config."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-112-{int(time.time())}"

        # Setup objectives with MuJoCo but include fluid objectives
        objectives_content = """
physics:
  backend: "mujoco"
objectives:
  goal_zone: {min: [10,10,10], max: [12,12,12]}
  build_zone: {min: [-100,-100,-100], max: [100,100,100]}
  fluid_objectives:
    - type: "fluid_containment"
      fluid_id: "water"
      containment_zone: {min: [-1, -1, -1], max: [1, 1, 1]}
      threshold: 0.9
simulation_bounds: {min: [-100,-100,-100], max: [100,100,100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0,0,0], runtime_jitter: [0,0,0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "objectives.yaml", "content": objectives_content},
            headers={"X-Session-ID": session_id},
        )

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "script.py",
                "content": """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "smoke_test_mode": True},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # Should succeed despite fluid config (as it's ignored)
        # We can't guarantee success because of target_box missing, but it shouldn't crash.
        # But wait, my script doesn't define target_box.
        # MuJoCo loop searches for target_box.
        # If not found, it just stable-simulates.

        fluid_metrics = data.get("artifacts", {}).get("fluid_metrics", [])
        assert len(fluid_metrics) == 0, "Fluid metrics should be empty for MuJoCo"
