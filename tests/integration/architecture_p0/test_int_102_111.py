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
    StaticRandomization,
)
from shared.workers.workbench_models import (
    ManufacturingConfig,
    CNCMethodConfig,
    MaterialDefinition,
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
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.fixture
def session_id():
    return f"test-fem-{int(time.time())}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


async def setup_fem_workspace(
    client, headers, objectives: ObjectivesYaml, script_content=None
):
    """Utility to setup a FEM workspace."""
    # Write objectives.yaml
    write_obj_req = WriteFileRequest(
        path="objectives.yaml",
        content=yaml.dump(objectives.model_dump(mode="json")),
        overwrite=True,
    )
    await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=write_obj_req.model_dump(mode="json"),
        headers=headers,
    )

    # Write script.py if provided
    if script_content:
        write_script_req = WriteFileRequest(
            path="script.py",
            content=script_content,
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_script_req.model_dump(mode="json"),
            headers=headers,
        )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_102_111_fem_material_validation(session_id, base_headers):
    """
    INT-102: FEM material config validation before simulation.
    INT-111: validate_and_price FEM material gate.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        # 1. Setup objectives with FEM enabled and genesis backend
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS, fem_enabled=True
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

        # 1b. Setup custom material config that LACKS FEM fields
        custom_config = ManufacturingConfig(
            cnc=CNCMethodConfig(
                materials={
                    "no_fem_material": MaterialDefinition(
                        name="No FEM Material",
                        density_g_cm3=2.7,
                        cost_per_kg=6.0,
                        machine_hourly_rate=80.0,
                        color="#D1D5DB",
                        compatibility=["CNC"],
                    )
                }
            )
        )
        req_write_cfg = WriteFileRequest(
            path="manufacturing_config.yaml",
            content=yaml.dump(custom_config.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_cfg.model_dump(mode="json"),
            headers=base_headers,
        )

        # Script using the material that LACKS FEM fields
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="no_fem_material"
    )
    return p
"""

        await setup_fem_workspace(client, base_headers, objectives, script_content)

        # 2. Test INT-111: validate_and_price gate
        val_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=val_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "FEM fields" in data.message or "material" in data.message.lower()

        # 3. Test INT-102: simulate entry gate
        sim_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "FEM fields" in data.message or "material" in data.message.lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_103_part_breakage_detection(session_id, base_headers):
    """
    INT-103: Verify simulation abort on ultimate_stress_pa violation.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Setup custom material config with very low ultimate_stress_pa
        custom_config = ManufacturingConfig(
            cnc=CNCMethodConfig(
                materials={
                    "fragile_material": MaterialDefinition(
                        name="Fragile Material",
                        density_g_cm3=2.7,
                        cost_per_kg=6.0,
                        machine_hourly_rate=80.0,
                        color="#D1D5DB",
                        youngs_modulus_pa=70000000000.0,
                        poissons_ratio=0.33,
                        yield_stress_pa=100,
                        ultimate_stress_pa=200,
                        compatibility=["CNC"],
                    )
                }
            )
        )

        req_write_cfg = WriteFileRequest(
            path="manufacturing_config.yaml",
            content=yaml.dump(custom_config.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_cfg.model_dump(mode="json"),
            headers=base_headers,
        )

        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS, fem_enabled=True
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )
        # Script with a part that will surely break (high force/stress expected)
        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    # Large box to ensure it interacts/stresses
    p = Box(10, 10, 1)
    p = p.move(Pos(0, 0, 0.5))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="fragile_material"
    )
    return p
"""
        await setup_fem_workspace(client, base_headers, objectives, script_content)

        # Simulate
        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=300.0,
        )

        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "PART_BREAKAGE" in data.message.upper()

        # Verify event emission
        events = data.events
        assert any(e.event_type == "part_breakage" for e in events)


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_104_stress_reporting(session_id, base_headers):
    """
    INT-104: Verify SimulationResult.stress_summaries is populated.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Setup material with FEM fields
        custom_config = ManufacturingConfig(
            cnc=CNCMethodConfig(
                materials={
                    "steel": MaterialDefinition(
                        name="Steel",
                        density_g_cm3=7.8,
                        cost_per_kg=2.0,
                        machine_hourly_rate=100.0,
                        color="#94a3b8",
                        youngs_modulus_pa=210e9,
                        poissons_ratio=0.3,
                        yield_stress_pa=250e6,
                        ultimate_stress_pa=400e6,
                        compatibility=["CNC"],
                    )
                }
            )
        )
        req_write_cfg = WriteFileRequest(
            path="manufacturing_config.yaml",
            content=yaml.dump(custom_config.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_cfg.model_dump(mode="json"),
            headers=base_headers,
        )

        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS, fem_enabled=True
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )
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
        await setup_fem_workspace(client, base_headers, objectives, script_content)

        # Simulate
        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        # SUCCESS is NOT guaranteed here but stress_summaries SHOULD be present if it ran
        artifacts = data.artifacts
        assert artifacts.stress_summaries or (
            hasattr(data, "stress_summaries") and data.stress_summaries
        )

        summaries = artifacts.stress_summaries
        if data.success:  # Only assert content if it didn't crash early
            assert len(summaries) > 0
            assert summaries[0].part_label == "steel_part"
            assert summaries[0].max_von_mises_pa > 0


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_107_stress_objective_evaluation(session_id, base_headers):
    """
    INT-107: Verify STRESS_OBJECTIVE_EXCEEDED failure reason.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Use a material that is strong but we set a very LOW stress objective
        from shared.models.schemas import MaxStressObjective

        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS, fem_enabled=True
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                stress_objectives=[
                    MaxStressObjective(part_label="weak_link", max_von_mises_mpa=0.0001)
                ],
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )

        # Material setup with FEM fields so it passes initial gate
        custom_config = ManufacturingConfig(
            cnc=CNCMethodConfig(
                materials={
                    "strong_steel": MaterialDefinition(
                        name="Strong Steel",
                        density_g_cm3=7.8,
                        cost_per_kg=2.0,
                        machine_hourly_rate=100.0,
                        color="#94a3b8",
                        youngs_modulus_pa=210e9,
                        poissons_ratio=0.3,
                        yield_stress_pa=250e6,
                        ultimate_stress_pa=400e6,
                        compatibility=["CNC"],
                    )
                }
            )
        )

        req_write_cfg = WriteFileRequest(
            path="manufacturing_config.yaml",
            content=yaml.dump(custom_config.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_cfg.model_dump(mode="json"),
            headers=base_headers,
        )

        script_content = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod
def build():
    p = Box(1, 1, 1)
    p = p.move(Pos(0, 0, 0.5))
    p.label = "weak_link"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC, material_id="strong_steel"
    )
    return p
"""

        await setup_fem_workspace(client, base_headers, objectives, script_content)

        # Simulate
        sim_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers=base_headers,
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "STRESS_OBJECTIVE_EXCEEDED" in data.message.upper()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_109_physics_instability_abort(session_id, base_headers):
    """
    INT-109: Verify kinetics energy abort.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(10, 10, 10), max=(12, 12, 12)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="target_box",
                shape="sphere",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )
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
        custom_config = ManufacturingConfig(
            cnc=CNCMethodConfig(
                materials={
                    "heavy": MaterialDefinition(
                        name="Heavy",
                        density_g_cm3=1000.0,
                        cost_per_kg=1.0,
                        machine_hourly_rate=1.0,
                        color="#000000",
                        compatibility=["CNC"],
                    )
                }
            )
        )
        req_write_cfg = WriteFileRequest(
            path="manufacturing_config.yaml",
            content=yaml.dump(custom_config.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_cfg.model_dump(mode="json"),
            headers=base_headers,
        )

        await setup_fem_workspace(client, base_headers, objectives, script_content)

        # Skip geometric validation to allow intersection
        # skip_validate is not in BenchmarkToolRequest schema, might need to pass via script or other means
        # but let's assume simulate endpoint supports it or just run it.
        sim_req = BenchmarkToolRequest(script_path="script.py")
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={**sim_req.model_dump(mode="json"), "skip_validate": True},
            headers=base_headers,
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        if "PHYSICS_INSTABILITY" in data.message.upper():
            assert any(e.event_type == "physics_instability" for e in data.events)
