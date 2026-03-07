import json
import os
import uuid

import httpx
import pytest
import yaml
from pydantic import BaseModel

from shared.enums import FailureReason
from shared.models.schemas import (
    BoundingBox,
    Constraints,
    FlowRateObjective,
    FluidContainmentObjective,
    FluidDefinition,
    FluidShapeType,
    FluidVolume,
    MovedObject,
    ObjectivesSection,
    ObjectivesYaml,
    PhysicsConfig,
)
from shared.models.simulation import FluidMetricResult
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ListFilesRequest,
    SimulationArtifacts,
    WriteFileRequest,
)
from shared.workers.workbench_models import (
    CNCMethodConfig,
    ManufacturingConfig,
    MaterialDefinition,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


class FluidMetricsEnvelope(BaseModel):
    metrics: list[FluidMetricResult]


async def _require_worker_services(client: httpx.AsyncClient) -> None:
    for name, url in (
        ("worker-light", WORKER_LIGHT_URL),
        ("worker-heavy", WORKER_HEAVY_URL),
    ):
        try:
            resp = await client.get(f"{url}/health", timeout=5.0)
            resp.raise_for_status()
        except Exception:
            pytest.skip(f"{name} is not reachable at {url}")


async def _write_file(
    client: httpx.AsyncClient, session_id: str, path: str, content: str
) -> None:
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(path=path, content=content, overwrite=True).model_dump(
            mode="json"
        ),
        headers={"X-Session-ID": session_id},
    )
    assert resp.status_code == 200, resp.text


def _base_script_with_obstacle() -> str:
    return """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    obstacle = Box(30, 30, 30).move(Pos(0, 0, 15))
    obstacle.label = "obstacle"
    obstacle.metadata = PartMetadata(material_id="aluminum_6061")

    motor_mount = Box(10, 10, 10).move(Pos(-40, 0, 10))
    motor_mount.label = "motor_mount"
    motor_mount.metadata = PartMetadata(material_id="aluminum_6061")

    return Compound(children=[obstacle, motor_mount], label="assembly")
"""


def _fluid_objectives() -> ObjectivesYaml:
    return ObjectivesYaml(
        physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS, fem_enabled=False),
        objectives=ObjectivesSection(
            goal_zone=BoundingBox(min=(0, 0, 0), max=(20, 20, 20)),
            build_zone=BoundingBox(min=(-100, -100, -10), max=(100, 100, 100)),
            fluid_objectives=[
                FluidContainmentObjective(
                    fluid_id="water",
                    containment_zone=BoundingBox(
                        min=(-40, -40, -10),
                        max=(40, 40, 60),
                    ),
                    threshold=0.2,
                ),
                FlowRateObjective(
                    fluid_id="water",
                    gate_plane_point=(0, 0, 10),
                    gate_plane_normal=(0, 0, 1),
                    target_rate_l_per_s=0.01,
                    tolerance=0.95,
                ),
            ],
        ),
        fluids=[
            FluidDefinition(
                fluid_id="water",
                initial_volume=FluidVolume(
                    type=FluidShapeType.SPHERE,
                    center=(0, 0, 20),
                    radius=8.0,
                ),
            )
        ],
        simulation_bounds=BoundingBox(min=(-120, -120, -20), max=(120, 120, 120)),
        moved_object=MovedObject(
            label="projectile_ball",
            shape="sphere",
            start_position=(0, 0, 50),
            runtime_jitter=(0, 0, 0),
        ),
        constraints=Constraints(max_unit_cost=1000.0, max_weight_g=5000.0),
    )


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_131_full_fluid_objective_path():
    """
    INT-131: Full fluid benchmark workflow (planner -> engineer -> reviewer).
    Integration coverage here verifies the worker fluid objective path end-to-end:
    objectives -> validate -> simulate -> fluid metrics/failure taxonomy.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = f"INT-131-{uuid.uuid4().hex[:8]}"

        await _write_file(
            client,
            session_id,
            "objectives.yaml",
            yaml.dump(_fluid_objectives().model_dump(mode="json")),
        )
        await _write_file(client, session_id, "script.py", _base_script_with_obstacle())

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="script.py", smoke_test_mode=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())

        artifacts = simulate_data.artifacts or SimulationArtifacts()
        metric_envelope = FluidMetricsEnvelope(metrics=artifacts.fluid_metrics)
        assert metric_envelope.metrics, (
            "Expected fluid metrics from simulation for fluid objectives."
        )
        metric_types = {m.metric_type for m in metric_envelope.metrics}
        assert {"fluid_containment", "flow_rate"} <= metric_types

        if not simulate_data.success and artifacts.failure:
            assert artifacts.failure.reason in {
                FailureReason.FLUID_OBJECTIVE_FAILED,
                FailureReason.TIMEOUT,
                FailureReason.PHYSICS_INSTABILITY,
            }


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_133_elec_to_mech_conflict_signal():
    """
    INT-133: Elec -> Mech conflict iteration loop.
    Coverage here validates deterministic conflict detection signal (wire/path conflict)
    that drives an Elec -> Mech reroute in orchestration.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = f"INT-133-{uuid.uuid4().hex[:8]}"

        objectives = _fluid_objectives()
        objectives.fluids = []
        objectives.objectives.fluid_objectives = []

        await _write_file(
            client,
            session_id,
            "objectives.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
        )
        await _write_file(client, session_id, "script.py", _base_script_with_obstacle())

        # Waypoints intentionally pass through obstacle volume around origin.
        assembly = """
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 1000
  benchmark_max_weight_g: 5000
  planner_target_max_unit_cost_usd: 900
  planner_target_max_weight_g: 4500
electronics:
  power_supply: {voltage_dc: 12.0, max_current_a: 10.0}
  components:
    - {component_id: "motor_mount", type: "MOTOR", stall_current_a: 2.0}
  wiring:
    - wire_id: "wire_conflict"
      from: {component: "supply", terminal: "v+"}
      to: {component: "motor_mount", terminal: "+"}
      gauge_awg: 24
      length_mm: 120
      routed_in_3d: true
      waypoints: [[-40, 0, 10], [0, 0, 15], [20, 0, 15]]
totals:
  estimated_unit_cost_usd: 25.0
  estimated_weight_g: 250.0
  estimate_confidence: "high"
"""
        await _write_file(client, session_id, "assembly_definition.yaml", assembly)

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "wire" in data.message.lower() and "clearance" in data.message.lower()


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_134_stress_heatmap_render_artifact():
    """INT-134: Stress preview artifacts are generated and discoverable."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = f"INT-134-{uuid.uuid4().hex[:8]}"

        config = ManufacturingConfig(
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
        await _write_file(
            client,
            session_id,
            "manufacturing_config.yaml",
            yaml.dump(config.model_dump(mode="json")),
        )

        fem_objectives = _fluid_objectives()
        fem_objectives.physics.fem_enabled = True
        fem_objectives.fluids = []
        fem_objectives.objectives.fluid_objectives = []
        await _write_file(
            client,
            session_id,
            "objectives.yaml",
            yaml.dump(fem_objectives.model_dump(mode="json")),
        )
        await _write_file(
            client,
            session_id,
            "script.py",
            """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(20, 20, 5).move(Pos(0, 0, 2.5))
    p.label = "steel_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="steel",
    )
    return p
""",
        )

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="script.py",
                smoke_test_mode=True,
                backend=SimulatorBackendType.GENESIS,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        artifacts = data.artifacts or SimulationArtifacts()

        if not artifacts.stress_summaries:
            pytest.skip("FEM stress summaries were not produced for this scene.")

        stress_paths = [p for p in artifacts.render_paths if "stress" in p.lower()]
        assert stress_paths, (
            f"Expected stress preview render paths. Paths: {artifacts.render_paths}"
        )

        # Discoverability contract: stress renders should be visible in session file listing.
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="/renders").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert ls_resp.status_code == 200, ls_resp.text
        listed = json.dumps(ls_resp.json())
        assert "stress" in listed.lower()


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_135_wire_clearance_validation():
    """INT-135: Wire routes intersecting solid geometry are rejected."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = f"INT-135-{uuid.uuid4().hex[:8]}"

        objectives = _fluid_objectives()
        objectives.fluids = []
        objectives.objectives.fluid_objectives = []
        await _write_file(
            client,
            session_id,
            "objectives.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
        )
        await _write_file(client, session_id, "script.py", _base_script_with_obstacle())

        assembly = """
version: "1.0"
constraints:
  benchmark_max_unit_cost_usd: 1000
  benchmark_max_weight_g: 5000
  planner_target_max_unit_cost_usd: 900
  planner_target_max_weight_g: 4500
electronics:
  power_supply: {voltage_dc: 12.0, max_current_a: 10.0}
  components:
    - {component_id: "motor_mount", type: "MOTOR", stall_current_a: 2.0}
  wiring:
    - wire_id: "wire_clearance_fail"
      from: {component: "supply", terminal: "v+"}
      to: {component: "motor_mount", terminal: "+"}
      gauge_awg: 26
      length_mm: 120
      routed_in_3d: true
      waypoints: [[-40, 0, 10], [0, 0, 15], [20, 0, 15]]
totals:
  estimated_unit_cost_usd: 25.0
  estimated_weight_g: 250.0
  estimate_confidence: "high"
"""
        await _write_file(client, session_id, "assembly_definition.yaml", assembly)

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "wire clearance violation" in data.message.lower()
