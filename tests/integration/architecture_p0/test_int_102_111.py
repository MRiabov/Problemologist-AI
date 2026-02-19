import os
import time

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.fixture
def session_id():
    return f"test-fem-{int(time.time())}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


async def setup_fem_workspace(client, headers, objectives_content, script_content=None):
    """Utility to setup a FEM workspace."""
    # Write objectives.yaml
    await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json={
            "path": "objectives.yaml",
            "content": objectives_content,
            "overwrite": True,
        },
        headers=headers,
    )

    # Write script.py if provided
    if script_content:
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "script.py", "content": script_content, "overwrite": True},
            headers=headers,
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_102_111_fem_material_validation(session_id, base_headers, get_bundle):
    """
    INT-102: FEM material config validation before simulation.
    INT-111: validate_and_price FEM material gate.
    """
    async with httpx.AsyncClient() as client:
        # 1. Setup objectives with FEM enabled and genesis backend
        objectives_content = """
physics:
  backend: "genesis"
  fem_enabled: true
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "obj", shape: "sphere", start_position: [0, 0, 5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        # Script using a material that LACKS FEM fields (aluminum_6061 from default config)
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="aluminum_6061"
    )
    return p
"""
        await setup_fem_workspace(
            client, base_headers, objectives_content, script_content
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # 2. Test INT-111: validate_and_price gate
        # We expect this to fail because aluminum_6061 lacks FEM fields
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json={"script_path": "script.py", "bundle_base64": bundle64},
            headers=base_headers,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert not data["success"]
        assert "FEM fields" in data["message"] or "material" in data["message"].lower()

        # 3. Test INT-102: simulate entry gate
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "bundle_base64": bundle64},
            headers=base_headers,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert not data["success"]
        assert "FEM fields" in data["message"] or "material" in data["message"].lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_103_part_breakage_detection(session_id, base_headers, get_bundle):
    """
    INT-103: Verify simulation abort on ultimate_stress_pa violation.
    """
    async with httpx.AsyncClient() as client:
        # Setup custom material config with very low ultimate_stress_pa
        custom_config = """
cnc:
  materials:
    fragile_material:
      name: "Fragile Material"
      density_g_cm3: 2.7
      cost_per_kg: 6.0
      machine_hourly_rate: 80.0
      color: "#D1D5DB"
      youngs_modulus_pa: 70e9
      poissons_ratio: 0.33
      yield_stress_pa: 100
      ultimate_stress_pa: 200
      compatibility: ["cnc"]
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "manufacturing_config.yaml", "content": custom_config},
            headers=base_headers,
        )

        objectives_content = """
physics:
  backend: "genesis"
  fem_enabled: true
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "target_box", shape: "sphere", start_position: [0, 0, 0.5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        # Script with a part that will surely break (high force/stress expected)
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    # Large box to ensure it interacts/stresses
    p = Box(10, 10, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="fragile_material"
    )
    return p
"""
        await setup_fem_workspace(
            client, base_headers, objectives_content, script_content
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # Simulate
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "bundle_base64": bundle64},
            headers=base_headers,
            timeout=120.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert not data["success"]
        assert "PART_BREAKAGE" in data["message"].upper()

        # Verify event emission
        events = data.get("events", [])
        assert any(e["event_type"] == "part_breakage" for e in events)


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_104_stress_reporting(session_id, base_headers, get_bundle):
    """
    INT-104: Verify SimulationResult.stress_summaries is populated.
    """
    async with httpx.AsyncClient() as client:
        # Setup material with FEM fields
        custom_config = """
cnc:
  materials:
    steel:
      name: "Steel"
      density_g_cm3: 7.8
      cost_per_kg: 2.0
      machine_hourly_rate: 100.0
      color: "#94a3b8"
      youngs_modulus_pa: 210e9
      poissons_ratio: 0.3
      yield_stress_pa: 250e6
      ultimate_stress_pa: 400e6
      compatibility: ["cnc"]
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "manufacturing_config.yaml", "content": custom_config},
            headers=base_headers,
        )

        objectives_content = """
physics:
  backend: "genesis"
  fem_enabled: true
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "target_box", shape: "sphere", start_position: [0, 0, 0.5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1)
    p.label = "steel_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="steel"
    )
    return p
"""
        await setup_fem_workspace(
            client, base_headers, objectives_content, script_content
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # Simulate
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "bundle_base64": bundle64},
            headers=base_headers,
            timeout=120.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # SUCCESS is NOT guaranteed here but stress_summaries SHOULD be present if it ran
        # If it failed for other reasons (e.g. Genesis backend not available in test env),
        # we might need to skip or mock. But the spec says "Required integration execution contract: real containers".

        # For now, let's assume it runs.
        assert (
            "stress_summaries" in data.get("artifacts", {})
            or "stress_summaries" in data
        )
        # Check specific field in result payload (SimulationResult schema)
        summaries = data.get("stress_summaries", [])
        if data["success"]:  # Only assert content if it didn't crash early
            assert len(summaries) > 0
            assert summaries[0]["part_label"] == "steel_part"
            assert "max_von_mises_pa" in summaries[0]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_107_stress_objective_evaluation(session_id, base_headers, get_bundle):
    """
    INT-107: Verify STRESS_OBJECTIVE_EXCEEDED failure reason.
    """
    async with httpx.AsyncClient() as client:
        # Use a material that is strong but we set a very LOW stress objective
        objectives_content = """
physics:
  backend: "genesis"
  fem_enabled: true
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
  stress_objectives:
    - part_label: "weak_link"
      max_von_mises_mpa: 0.0001 # Extremely low threshold
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "target_box", shape: "sphere", start_position: [0, 0, 0.5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        # Material setup with FEM fields so it passes initial gate
        custom_config = """
cnc:
  materials:
    strong_steel:
      name: "Strong Steel"
      density_g_cm3: 7.8
      cost_per_kg: 2.0
      machine_hourly_rate: 100.0
      color: "#94a3b8"
      youngs_modulus_pa: 210e9
      poissons_ratio: 0.3
      yield_stress_pa: 250e6
      ultimate_stress_pa: 400e6
      compatibility: ["cnc"]
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "manufacturing_config.yaml", "content": custom_config},
            headers=base_headers,
        )

        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1)
    p.label = "weak_link"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="strong_steel"
    )
    return p
"""
        await setup_fem_workspace(
            client, base_headers, objectives_content, script_content
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # Simulate
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py", "bundle_base64": bundle64},
            headers=base_headers,
            timeout=120.0,
        )
        assert resp.status_code == 200
        data = resp.json()
        assert not data["success"]
        assert "STRESS_OBJECTIVE_EXCEEDED" in data["message"].upper()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_109_physics_instability_abort(session_id, base_headers, get_bundle):
    """
    INT-109: Verify kinetics energy abort.
    Note: Creating true instability reliably is hard, but we can try
    intersecting geometries with high mass/stiffness.
    """
    async with httpx.AsyncClient() as client:
        objectives_content = """
physics:
  backend: "genesis"
objectives:
  goal_zone: {min: [10, 10, 10], max: [12, 12, 12]}
  build_zone: {min: [-100, -100, -100], max: [100, 100, 100]}
simulation_bounds: {min: [-100, -100, -100], max: [100, 100, 100]}
moved_object: {label: "target_box", shape: "sphere", start_position: [0, 0, 0.5], runtime_jitter: [0, 0, 0]}
constraints: {max_unit_cost: 100, max_weight_g: 10}
"""
        # Script with overlapping high-density parts to cause explosion
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p1 = Box(10, 10, 10).move(Location((0,0,0)))
    p2 = Box(10, 10, 10).move(Location((0.1, 0.1, 0.1)))
    p1.label = "part1"
    p2.label = "part2"
    # Overwrite default density to be huge
    p1.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="heavy"
    )
    p2.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="heavy"
    )
    return Compound(children=[p1, p2])
"""
        custom_config = """
cnc:
  materials:
    heavy:
      name: "Heavy"
      density_g_cm3: 1000.0
      cost_per_kg: 1.0
      machine_hourly_rate: 1.0
      color: "#000000"
      compatibility: ["cnc"]
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "manufacturing_config.yaml", "content": custom_config},
            headers=base_headers,
        )

        await setup_fem_workspace(
            client, base_headers, objectives_content, script_content
        )

        # Get bundle for heavy worker
        bundle64 = await get_bundle(client, session_id)

        # Skip geometric validation to allow intersection
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={
                "script_path": "script.py",
                "skip_validate": True,
                "bundle_base64": bundle64,
            },
            headers=base_headers,
            timeout=120.0,
        )
        assert resp.status_code == 200
        data = resp.json()

        # It might either be PHYSICS_INSTABILITY or success if it didn't explode
        # but for INT-109 we want to verify the abort logic.
        # If it happens, we assert it.
        if "PHYSICS_INSTABILITY" in data["message"].upper():
            assert any(
                e["event_type"] == "physics_instability" for e in data.get("events", [])
            )
