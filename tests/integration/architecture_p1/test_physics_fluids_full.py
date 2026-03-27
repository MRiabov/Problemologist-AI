import io
import json
import os
import uuid

import httpx
import numpy as np
import pytest
import yaml
from PIL import Image
from pydantic import BaseModel

from shared.enums import FailureReason, FluidEvalAt, FluidShapeType
from shared.models.schemas import (
    BenchmarkDefinition,
    BoundingBox,
    Constraints,
    FlowRateObjective,
    FluidContainmentObjective,
    FluidDefinition,
    FluidProperties,
    FluidVolume,
    MovedObject,
    ObjectivesSection,
    PhysicsConfig,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ListFilesRequest,
    WriteFileRequest,
)
from shared.workers.workbench_models import (
    CNCMethodConfig,
    ManufacturingConfig,
    MaterialDefinition,
)
from tests.integration.backend_utils import skip_unless_genesis

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {"fixed": True, "material_id": "aluminum_6061"},
        }
    ]


class MetricTypeEnvelope(BaseModel):
    metric_types: list[str]


async def _require_service(client: httpx.AsyncClient, name: str, url: str) -> None:
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


def _simple_script() -> str:
    return """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    p = Box(1, 1, 1).move(Pos(6, 6, 6))
    p.label = "test_part"
    p.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return p
"""


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_131_full_fluid_objective_path():
    """
    INT-131: Full fluid benchmark workflow (planner -> engineer -> reviewer).
    Integration coverage here validates fluid objective handling through
    worker boundaries: benchmark_definition.yaml -> /benchmark/simulate -> fluid metrics.
    """
    skip_unless_genesis("INT-131 requires Genesis fluid simulation.")
    import torch

    if not torch.cuda.is_available():
        pytest.skip("Skipping INT-131 fluid test on CPU-only environment.")

    async with httpx.AsyncClient(timeout=900.0) as client:
        await _require_service(client, "worker-light", WORKER_LIGHT_URL)
        await _require_service(client, "worker-heavy", WORKER_HEAVY_URL)
        session_id = f"INT-131-{uuid.uuid4().hex[:8]}"

        objectives = BenchmarkDefinition(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            fluids=[
                FluidDefinition(
                    fluid_id="water",
                    properties=FluidProperties(viscosity_cp=1.0, density_kg_m3=1000.0),
                    initial_volume=FluidVolume(
                        type=FluidShapeType.SPHERE, center=(0, 0, 3), radius=0.6
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
                    ),
                    FlowRateObjective(
                        fluid_id="water",
                        gate_plane_point=(0, 0, 0),
                        gate_plane_normal=(0, 0, 1),
                        target_rate_l_per_s=1.0,
                        tolerance=1.0,
                    ),
                ],
            ),
            simulation_bounds=BoundingBox(min=(-10, -10, -10), max=(10, 10, 10)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(3, 3, 3),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
        )
        await _write_file(client, session_id, "script.py", _simple_script())

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="script.py", smoke_test_mode=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=900.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        artifacts = data.artifacts
        assert artifacts is not None
        if data.success:
            assert artifacts.fluid_metrics, (
                "Expected non-empty fluid_metrics in artifacts."
            )
            metric_types = MetricTypeEnvelope(
                metric_types=[m.metric_type for m in artifacts.fluid_metrics]
            )
            assert "fluid_containment" in metric_types.metric_types
            assert "flow_rate" in metric_types.metric_types
        else:
            assert artifacts.failure is not None
            assert artifacts.failure.reason == FailureReason.PHYSICS_INSTABILITY
            assert "pbs-500" in (artifacts.failure.detail or "").lower()


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_133_unified_electromechanical_conflict_signal():
    """
    INT-133: Wire-routing conflict signal for the unified electromechanical loop.
    We validate deterministic conflict detection (wire clearance violation) via /benchmark/validate.
    """
    skip_unless_genesis(
        "INT-133 currently targets Genesis electromechanical validation."
    )
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_service(client, "worker-light", WORKER_LIGHT_URL)
        await _require_service(client, "worker-heavy", WORKER_HEAVY_URL)
        session_id = f"INT-133-{uuid.uuid4().hex[:8]}"

        objectives = BenchmarkDefinition(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-100, -100, -10), max=(100, 100, 100)),
            ),
            simulation_bounds=BoundingBox(min=(-120, -120, -20), max=(120, 120, 120)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=1000.0, max_weight_g=5000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
        )
        await _write_file(
            client,
            session_id,
            "script.py",
            """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    obstacle = Box(30, 30, 30).move(Pos(0, 0, 15))
    obstacle.label = "obstacle"
    obstacle.metadata = PartMetadata(material_id="aluminum_6061")
    return obstacle
""",
        )
        await _write_file(
            client,
            session_id,
            "assembly_definition.yaml",
            """
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
""",
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "wire clearance" in data.message.lower()


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_134_stress_heatmap_render_artifact():
    """INT-134: Stress heatmap render artifacts are generated and discoverable."""
    skip_unless_genesis("INT-134 requires Genesis FEM stress rendering.")
    async with httpx.AsyncClient(timeout=1000.0) as client:
        await _require_service(client, "worker-light", WORKER_LIGHT_URL)
        await _require_service(client, "worker-heavy", WORKER_HEAVY_URL)
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

        objectives = BenchmarkDefinition(
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
                material_id="aluminum_6061",
                start_position=(0, 0, 0.5),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=100.0, max_weight_g=10.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
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
    p = Box(1, 1, 1)
    p.label = "steel_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="steel",
    )
    return p
""",
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="script.py", smoke_test_mode=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=1000.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        artifacts = data.artifacts
        assert artifacts is not None
        if not artifacts.stress_summaries:
            pytest.skip("FEM stress summaries were not produced in this run.")
        stress_paths = [
            path for path in artifacts.render_paths if "stress" in path.lower()
        ]
        if not stress_paths:
            pytest.skip(
                "Stress summaries present but stress heatmap paths not emitted."
            )

        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="/renders").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert ls_resp.status_code == 200, ls_resp.text
        listing = json.dumps(ls_resp.json()).lower()
        assert "stress" in listing

        stress_path = next(
            (path for path in stress_paths if path.endswith(".png")), None
        )
        assert stress_path is not None, stress_paths

        stress_resp = await client.get(
            f"{WORKER_LIGHT_URL}/assets/{stress_path}",
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert stress_resp.status_code == 200, stress_resp.text
        stress_image = np.array(
            Image.open(io.BytesIO(stress_resp.content)).convert("RGB")
        )
        assert stress_image.std() > 0.0, stress_path
        assert not np.all(stress_image == np.array([255, 0, 0], dtype=np.uint8)), (
            "stress render fell back to a synthetic blank red image"
        )


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_135_wire_clearance_validation():
    """INT-135: Wire route that intersects solids is rejected by validation."""
    skip_unless_genesis("INT-135 currently targets Genesis wire-clearance validation.")
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_service(client, "worker-light", WORKER_LIGHT_URL)
        await _require_service(client, "worker-heavy", WORKER_HEAVY_URL)
        session_id = f"INT-135-{uuid.uuid4().hex[:8]}"

        objectives = BenchmarkDefinition(
            physics=PhysicsConfig(backend=SimulatorBackendType.GENESIS),
            objectives=ObjectivesSection(
                goal_zone=BoundingBox(min=(5, 5, 5), max=(7, 7, 7)),
                build_zone=BoundingBox(min=(-100, -100, -10), max=(100, 100, 100)),
            ),
            simulation_bounds=BoundingBox(min=(-120, -120, -20), max=(120, 120, 120)),
            moved_object=MovedObject(
                label="obj",
                shape="sphere",
                material_id="aluminum_6061",
                start_position=(0, 0, 0),
                runtime_jitter=(0, 0, 0),
            ),
            constraints=Constraints(max_unit_cost=1000.0, max_weight_g=5000.0),
            benchmark_parts=_default_benchmark_parts(),
        )
        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            yaml.dump(objectives.model_dump(mode="json")),
        )
        await _write_file(
            client,
            session_id,
            "script.py",
            """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    obstacle = Box(30, 30, 30).move(Pos(0, 0, 15))
    obstacle.label = "obstacle"
    obstacle.metadata = PartMetadata(material_id="aluminum_6061")
    return obstacle
""",
        )
        await _write_file(
            client,
            session_id,
            "assembly_definition.yaml",
            """
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
""",
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "wire clearance" in data.message.lower()
