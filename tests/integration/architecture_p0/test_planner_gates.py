import asyncio
import os
import uuid
import yaml
import pytest
import httpx

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:8001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:8000")


# Use a non-async fixture for session_id to avoid scope issues in some environments
@pytest.fixture
def session_id():
    return f"test-gates-{uuid.uuid4().hex[:8]}"


@pytest.fixture
def minimal_script():
    return """
from build123d import *
def build():
    return Box(10, 10, 10)
"""


@pytest.fixture
def base_objectives():
    return {
        "objectives": {
            "goal_zone": {"min": [100.0, 100.0, 100.0], "max": [110.0, 110.0, 110.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-50.0, -50.0, 0.0], "max": [50.0, 50.0, 100.0]},
        },
        "simulation_bounds": {
            "min": [-200.0, -200.0, 0.0],
            "max": [200.0, 200.0, 200.0],
        },
        "moved_object": {
            "label": "ball",
            "shape": "sphere",
            "start_position": [0.0, 0.0, 50.0],
            "runtime_jitter": [0.0, 0.0, 0.0],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight": 1.0},
    }


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_011_planner_caps_enforcement(
    session_id, minimal_script, base_objectives
):
    """
    INT-011: Verify handoff blockage when planner caps exceed benchmark limits.
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Setup workspace
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "solution.py", "content": minimal_script},
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": yaml.dump(base_objectives)},
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "plan.md",
                "content": "## 1. Solution Overview\n## 2. Parts List\n## 3. Assembly Strategy\n## 4. Cost & Weight Budget\n## 5. Risk Assessment",
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "todo.md", "content": "- [x] Done"},
            headers={"X-Session-ID": session_id},
        )

        # Create invalid cost estimation (target > benchmark cap)
        invalid_cost = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_kg": 1.0,
                "planner_target_max_unit_cost_usd": 60.0,  # OVER LIMIT
                "planner_target_max_weight_kg": 0.5,
            },
            "totals": {
                "estimated_unit_cost_usd": 40.0,
                "estimated_weight_g": 200.0,
                "estimate_confidence": "medium",
            },
        }
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "preliminary_cost_estimation.yaml",
                "content": yaml.dump(invalid_cost),
            },
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/submit",
            json={"script_path": "solution.py"},
            headers={"X-Session-ID": session_id},
        )

        data = resp.json()
        assert not data["success"], f"Expected failure but got success: {data}"
        assert "preliminary_cost_estimation.yaml invalid" in data["message"]
        assert (
            "Planner target cost (60.0) must be less than or equal to benchmark max cost (50.0)"
            in data["message"]
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_015_engineer_handover_immutability(
    session_id, minimal_script, base_objectives
):
    """
    INT-015: Verify immutability of objectives.yaml during handover.
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        await client.post(
            f"{WORKER_URL}/git/init", headers={"X-Session-ID": session_id}
        )

        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "solution.py", "content": minimal_script},
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": yaml.dump(base_objectives)},
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "plan.md",
                "content": "## 1. Solution Overview\n## 2. Parts List\n## 3. Assembly Strategy\n## 4. Cost & Weight Budget\n## 5. Risk Assessment",
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "todo.md", "content": "- [x] Done"},
            headers={"X-Session-ID": session_id},
        )

        valid_cost = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_kg": 1.0,
                "planner_target_max_unit_cost_usd": 45.0,
                "planner_target_max_weight_kg": 0.8,
            },
            "totals": {
                "estimated_unit_cost_usd": 40.0,
                "estimated_weight_g": 200.0,
                "estimate_confidence": "medium",
            },
        }
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "preliminary_cost_estimation.yaml",
                "content": yaml.dump(valid_cost),
            },
            headers={"X-Session-ID": session_id},
        )

        # Baseline commit
        await client.post(
            f"{WORKER_URL}/git/commit",
            json={"message": "Initial benchmark"},
            headers={"X-Session-ID": session_id},
        )

        # Cheat: modify objectives.yaml
        modified_objectives = base_objectives.copy()
        modified_objectives["constraints"]["max_unit_cost"] = 1000.0
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": yaml.dump(modified_objectives)},
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_URL}/benchmark/submit",
            json={"script_path": "solution.py"},
            headers={"X-Session-ID": session_id},
        )

        data = resp.json()
        assert not data["success"]
        assert "objectives.yaml violation" in data["message"]
        assert "has been modified" in data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_016_review_decision_schema_gate(session_id):
    """
    INT-016: Verify strict schema validation for reviewer decisions.
    We test this by running a small script on the worker that calls the validator.
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Invalid decision "maybe"
        invalid_review = """---
decision: maybe
comments:
  - This is not a valid decision
---
"""
        # Script to run validator
        validator_script = f"""
from worker.utils.file_validation import validate_review_frontmatter
content = {repr(invalid_review)}
is_valid, errors = validate_review_frontmatter(content)
print(f"VALID:{{is_valid}}")
print(f"ERRORS:{{errors}}")
"""
        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={"code": validator_script},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        stdout = resp.json()["stdout"]
        assert "VALID:False" in stdout
        assert (
            "decision" in stdout and "extra_forbidden" not in stdout
        )  # Decision should be in errors


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_019_hard_constraints_gates(session_id, base_objectives):
    """
    INT-019: Verify cost/weight/build-zone hard failure during submission.
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Setup valid baseline
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "objectives.yaml", "content": yaml.dump(base_objectives)},
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "plan.md",
                "content": "## 1. Solution Overview\n## 2. Parts List\n## 3. Assembly Strategy\n## 4. Cost & Weight Budget\n## 5. Risk Assessment",
            },
            headers={"X-Session-ID": session_id},
        )
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "todo.md", "content": "- [x] Done"},
            headers={"X-Session-ID": session_id},
        )

        valid_cost = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 50.0,
                "benchmark_max_weight_kg": 1.0,
                "planner_target_max_unit_cost_usd": 45.0,
                "planner_target_max_weight_kg": 0.8,
            },
            "totals": {
                "estimated_unit_cost_usd": 40.0,
                "estimated_weight_g": 200.0,
                "estimate_confidence": "medium",
            },
        }
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={
                "path": "preliminary_cost_estimation.yaml",
                "content": yaml.dump(valid_cost),
            },
            headers={"X-Session-ID": session_id},
        )

        # 1. Test OUT OF BOUNDS設計
        # Box translated to (1000, 1000, 1000) will be outside [ -50, 50 ]
        oob_script = """
from build123d import *
from worker.workbenches.models import ManufacturingMethod
def build():
    p = Box(10, 10, 10)
    p = p.move(Location((1000, 1000, 1000)))
    p.label = "oob_part"
    p.metadata = {"manufacturing_method": ManufacturingMethod.CNC, "material_id": "aluminum-6061"}
    return p
"""
        await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": "solution_oob.py", "content": oob_script},
            headers={"X-Session-ID": session_id},
        )
        resp = await client.post(
            f"{WORKER_URL}/benchmark/submit",
            json={"script_path": "solution_oob.py"},
            headers={"X-Session-ID": session_id},
        )
        data = resp.json()
        assert not data["success"]
        assert "Submission rejected (DFM)" in data["message"]
        assert "Build zone violation" in data["message"]
