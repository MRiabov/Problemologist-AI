import os
import uuid

import httpx
import pytest
import yaml

from shared.enums import AgentName
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ReadFileRequest,
    ReadFileResponse,
    WriteFileRequest,
)

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


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


async def _write_workspace_file(
    client: httpx.AsyncClient, headers: dict[str, str], path: str, content: str | dict
) -> None:
    payload = content if isinstance(content, str) else yaml.dump(content)
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path=path,
            content=payload,
            overwrite=True,
        ).model_dump(mode="json"),
        headers=headers,
    )
    assert resp.status_code == 200, f"Failed to write {path}: {resp.text}"


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_definition_yaml_invalid",
        "benchmark_definition_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_008_objectives_semantic_validation_rejects_goal_forbid_overlap():
    """
    INT-008: benchmark_definition.yaml validation must fail closed on semantic contradictions,
    not only schema errors, before benchmark submission proceeds.
    """
    session_id = f"INT-008-OBJ-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    minimal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    overlapping_objectives = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [
                {
                    "name": "out_of_bounds",
                    "min": [0.0, -2.0, 0.0],
                    "max": [3.0, 2.0, 2.0],
                }
            ],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "benchmark_parts": _default_benchmark_parts(),
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(client, headers, "solution.py", minimal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", overlapping_objectives
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert "goal_zone overlaps forbid zone" in data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_definition_yaml_invalid",
        "benchmark_definition_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_008_attachment_policy_rejects_legacy_attachment_method():
    """
    INT-008: benchmark_definition.yaml attachment policy must reject legacy
    attachment method values outside the typed fastener/none contract.
    """
    session_id = f"INT-008-ATT-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    minimal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    invalid_attachment_policy = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "benchmark_parts": [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["clamp"],
                        "notes": "Legacy invalid method",
                    },
                },
            }
        ],
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(client, headers, "solution.py", minimal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", invalid_attachment_policy
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "attachment_methods" in data.message
        assert "fastener" in data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_definition_yaml_invalid",
        "benchmark_definition_yaml_validation_error",
    ]
)
@pytest.mark.asyncio
async def test_int_008_requires_non_empty_benchmark_parts():
    """
    INT-008: benchmark_definition.yaml must fail closed when benchmark_parts is
    omitted because benchmark-owned fixtures are a required source contract.
    """
    session_id = f"INT-008-PARTS-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    minimal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    missing_benchmark_parts = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(client, headers, "solution.py", minimal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", missing_benchmark_parts
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers=headers,
        )
        assert resp.status_code == 200, resp.text
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "benchmark_parts" in data.message


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_008_submit_requires_explicit_reviewer_stage():
    """/benchmark/submit must fail closed when reviewer_stage is omitted."""
    session_id = f"INT-008-STAGE-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    minimal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    valid_objectives = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "benchmark_parts": _default_benchmark_parts(),
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(client, headers, "solution.py", minimal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", valid_objectives
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(script_path="solution.py").model_dump(
                mode="json"
            ),
            headers=headers,
        )
        assert resp.status_code == 400, resp.text
        assert "reviewer_stage is required" in resp.text


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "benchmark_definition_yaml_invalid",
        "environment_attachment_contract_invalid",
    ]
)
@pytest.mark.asyncio
async def test_int_008_interactable_benchmark_fixture_round_trips_permission():
    """
    INT-008: explicit interaction permission must survive YAML serialization and
    benchmark handoff validation.
    """
    session_id = f"INT-008-RT-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
environment_drill_operations:
  - target_part_id: environment_fixture
    hole_id: mount_left
    diameter_mm: 4.0
    depth_mm: 8.0
    quantity: 1
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    benchmark_assembly_definition = """version: "1.0"
units:
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
constraints:
  benchmark_max_unit_cost_usd: 50.0
  benchmark_max_weight_g: 1200.0
  planner_target_max_unit_cost_usd: 45.0
  planner_target_max_weight_g: 1000.0
manufactured_parts: []
cots_parts: []
environment_drill_operations:
  - target_part_id: environment_fixture
    hole_id: mount_left
    diameter_mm: 4.0
    depth_mm: 8.0
    quantity: 1
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: "medium"
dfm_suggestions: []
"""
    goal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    interactable_objectives = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "benchmark_parts": [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "allows_engineer_interaction": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "drill_policy": {
                            "allowed": True,
                            "max_hole_count": 1,
                            "diameter_range_mm": [3.0, 5.0],
                            "max_depth_mm": 10.0,
                        },
                    },
                },
            }
        ],
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(
            client,
            headers,
            "benchmark_assembly_definition.yaml",
            benchmark_assembly_definition,
        )
        await _write_workspace_file(client, headers, "script.py", goal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", interactable_objectives
        )

        read_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path="benchmark_definition.yaml").model_dump(
                mode="json"
            ),
            headers=headers,
        )
        assert read_resp.status_code == 200, read_resp.text
        benchmark_definition = yaml.safe_load(
            ReadFileResponse.model_validate(read_resp.json()).content
        )
        assert (
            benchmark_definition["benchmark_parts"][0]["metadata"][
                "allows_engineer_interaction"
            ]
            is True
        )

        benchmark_request = BenchmarkToolRequest(
            script_path="script.py",
            reviewer_stage=AgentName.BENCHMARK_REVIEWER,
        )
        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success, submit_data.message


@pytest.mark.integration_p0
@pytest.mark.allow_backend_errors(
    regexes=[
        "environment_attachment_contract_invalid",
    ]
)
@pytest.mark.asyncio
async def test_int_008_missing_interaction_permission_fails_closed_at_handoff():
    """
    INT-008: a benchmark-owned fixture that is used for engineer-facing
    interaction must fail closed when the explicit permission marker is absent.
    """
    session_id = f"INT-008-MISS-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    valid_plan = """## 1. Learning Objective

Move the projectile into the goal zone.

## 2. Geometry

- Ground plane
- Guide rails

## 3. Objectives

- Reach the goal zone
"""
    valid_todo = "# TODO\n\n- [x] Planner handoff seeded\n"
    valid_cost = """version: "1.0"
manufactured_parts: []
cots_parts: []
final_assembly: []
environment_drill_operations:
  - target_part_id: environment_fixture
    hole_id: mount_left
    diameter_mm: 4.0
    depth_mm: 8.0
    quantity: 1
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
"""
    benchmark_assembly_definition = """version: "1.0"
units:
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
constraints:
  benchmark_max_unit_cost_usd: 50.0
  benchmark_max_weight_g: 1200.0
  planner_target_max_unit_cost_usd: 45.0
  planner_target_max_weight_g: 1000.0
manufactured_parts: []
cots_parts: []
environment_drill_operations:
  - target_part_id: environment_fixture
    hole_id: mount_left
    diameter_mm: 4.0
    depth_mm: 8.0
    quantity: 1
final_assembly: []
totals:
  estimated_unit_cost_usd: 10.0
  estimated_weight_g: 100.0
  estimate_confidence: "medium"
dfm_suggestions: []
"""
    goal_script = """
from build123d import Box, Location
from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod

def build():
    p = Box(10, 10, 10)
    p = p.move(Location((0, 0, 5)))
    p.label = "test_part"
    p.metadata = PartMetadata(
        manufacturing_method=ManufacturingMethod.CNC,
        material_id="aluminum-6061",
    )
    return p
"""
    missing_permission_objectives = {
        "objectives": {
            "goal_zone": {"min": [1.0, -1.0, 0.0], "max": [2.0, 1.0, 1.0]},
            "forbid_zones": [],
            "build_zone": {"min": [-5.0, -5.0, 0.0], "max": [5.0, 5.0, 15.0]},
            "fluid_objectives": [],
            "stress_objectives": [],
        },
        "benchmark_parts": [
            {
                "part_id": "environment_fixture",
                "label": "environment_fixture",
                "metadata": {
                    "fixed": True,
                    "material_id": "aluminum_6061",
                    "attachment_policy": {
                        "attachment_methods": ["fastener"],
                        "drill_policy": {
                            "allowed": True,
                            "max_hole_count": 1,
                            "diameter_range_mm": [3.0, 5.0],
                            "max_depth_mm": 10.0,
                        },
                    },
                },
            }
        ],
        "physics": {"backend": "GENESIS", "fem_enabled": False},
        "fluids": [],
        "simulation_bounds": {"min": [-6.0, -6.0, 0.0], "max": [6.0, 6.0, 6.0]},
        "moved_object": {
            "label": "projectile_ball",
            "shape": "sphere",
            "material_id": "abs",
            "static_randomization": {"radius": [0.25, 0.25]},
            "start_position": [-4.0, 0.0, 0.5],
            "runtime_jitter": [0.1, 0.1, 0.1],
        },
        "constraints": {"max_unit_cost": 50.0, "max_weight_g": 1200.0},
        "randomization": {
            "static_variation_id": "v1.0",
            "runtime_jitter_enabled": True,
        },
    }

    async with httpx.AsyncClient(timeout=300.0) as client:
        await _write_workspace_file(client, headers, "plan.md", valid_plan)
        await _write_workspace_file(client, headers, "todo.md", valid_todo)
        await _write_workspace_file(
            client, headers, "assembly_definition.yaml", valid_cost
        )
        await _write_workspace_file(
            client,
            headers,
            "benchmark_assembly_definition.yaml",
            benchmark_assembly_definition,
        )
        await _write_workspace_file(client, headers, "script.py", goal_script)
        await _write_workspace_file(
            client, headers, "benchmark_definition.yaml", missing_permission_objectives
        )

        benchmark_request = BenchmarkToolRequest(
            script_path="script.py",
            reviewer_stage=AgentName.BENCHMARK_REVIEWER,
        )
        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success, simulate_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=benchmark_request.model_dump(mode="json"),
            headers=headers,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert not submit_data.success
        assert "allows_engineer_interaction" in submit_data.message
