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
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    FluidContainmentObjective,
    FlowRateObjective,
)
from shared.workers.schema import (
    BenchmarkToolResponse,
    BenchmarkToolRequest,
    WriteFileRequest,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.enums import FluidShapeType, FluidEvalAt

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
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS,
                fem_enabled=True,
                compute_target="cpu",
            ),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="test_obj",
                shape="sphere",
                start_position=(0, 0, 5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )
        write_obj_req = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_obj_req.model_dump(mode="json"),
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
        write_script_req = WriteFileRequest(
            path="script.py", content=script, overwrite=True
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_script_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # 3. Simulate
        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        events = data.events

        # Verify event emission
        assert any(e.event_type == "simulation_backend_selected" for e in events), (
            "Missing simulation_backend_selected event"
        )

        event = next(e for e in events if e.event_type == "simulation_backend_selected")
        assert event.backend == "genesis"
        assert event.fem_enabled is True
        assert event.compute_target == "cpu"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_105_fluid_containment_evaluation():
    """INT-105: Verify fluid containment objective evaluation."""
    import torch

    if not torch.cuda.is_available():
        pytest.skip("Skipping Genesis MPM test on CPU due to performance constraints.")

    async with httpx.AsyncClient(timeout=900.0) as client:
        session_id = f"test-int-105-{int(time.time())}"

        # Setup objectives with fluid containment
        # Zone contains (0,0,0)
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            fluids=[
                FluidDefinition(
                    fluid_id="water",
                    properties=FluidProperties(viscosity_cp=1.0, density_kg_m3=1000.0),
                    initial_volume=FluidVolume(
                        type=FluidShapeType.SPHERE, center=(0, 0, 0), radius=1.0
                    ),
                )
            ],
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                fluid_objectives=[
                    FluidContainmentObjective(
                        fluid_id="water",
                        containment_zone=BoundingBox(min=(-5, -5, -5), max=(5, 5, 5)),
                        threshold=0.95,
                        eval_at=FluidEvalAt.END,
                    )
                ],
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
        )

        write_obj_req = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=write_obj_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        req_write_script = WriteFileRequest(
            path="script.py",
            content="""
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success, f"Simulation failed: {data.message}"

        # Verify fluid metrics in result
        artifacts = data.artifacts
        fluid_metrics = artifacts.fluid_metrics
        assert len(fluid_metrics) > 0
        metric = fluid_metrics[0]
        assert metric.metric_type == "fluid_containment"
        assert metric.fluid_id == "water"
        # Since dummy particles are at (0,0,0) and zone is [-5, 5], ratio should be 1.0
        assert metric.measured_value == 1.0
        assert metric.passed is True

        # 4. Fail path: threshold > 1.0 (impossible to achieve)
        objectives.objectives.fluid_objectives[0].threshold = 1.1
        req_write_obj_fail = WriteFileRequest(
            path="objectives.yaml",
            content=yaml.dump(objectives.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_obj_fail.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert not data.success
        assert "FLUID_OBJECTIVE_FAILED" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_106_flow_rate_evaluation():
    """INT-106: Verify flow rate objective evaluation (stub)."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"test-int-106-{int(time.time())}"

        objectives = ObjectivesYaml(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                fluid_objectives=[
                    FlowRateObjective(
                        fluid_id="water",
                        gate_plane_point=(0, 0, 0),
                        gate_plane_normal=(0, 0, 1),
                        target_rate_l_per_s=1.0,
                    )
                ],
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
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

        req_write_script = WriteFileRequest(
            path="script.py",
            content="""
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
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
        objectives = ObjectivesYaml(
            physics=PhysicsConfig(
                backend=SimulatorBackendType.GENESIS
            ),  # Genesis but we check MuJoCo behavior if we toggle it
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
                fluid_objectives=[
                    FluidContainmentObjective(
                        fluid_id="water",
                        containment_zone=BoundingBox(min=(-1, -1, -1), max=(1, 1, 1)),
                        threshold=0.9,
                    )
                ],
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
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

        req_write_script = WriteFileRequest(
            path="script.py",
            content="""
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
""",
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        sim_req = BenchmarkToolRequest(
            script_path="script.py",
            smoke_test_mode=True,
            backend=SimulatorBackendType.MUJOCO,
        )
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())

        # Should succeed despite fluid config (as it's ignored)
        fluid_metrics = data.artifacts.fluid_metrics
        assert len(fluid_metrics) == 0, "Fluid metrics should be empty for MuJoCo"
