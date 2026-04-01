import json
import uuid

import httpx
import pytest
import yaml

from shared.enums import FailureReason
from shared.models.schemas import (
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    CostTotals,
    CotsPartEstimate,
    PartConfig,
)
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ExecuteRequest,
    ExecuteResponse,
    WriteFileRequest,
)
from tests.integration.backend_utils import selected_backend

WORKER_LIGHT_URL = "http://127.0.0.1:18001"
WORKER_HEAVY_URL = "http://127.0.0.1:18002"

pytestmark = pytest.mark.xdist_group(name="physics_sims")


async def _require_services(client: httpx.AsyncClient) -> None:
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


def _benchmark_definition_yaml() -> str:
    benchmark = BenchmarkDefinition.model_validate(
        {
            "physics": {"backend": selected_backend().value},
            "objectives": {
                "goal_zone": {"min": [20, -10, 0], "max": [40, 10, 20]},
                "build_zone": {"min": [-140, -140, -20], "max": [140, 140, 140]},
            },
            "simulation_bounds": {"min": [-160, -160, -40], "max": [160, 160, 160]},
            "moved_object": {
                "label": "moved_object",
                "shape": "sphere",
                "material_id": "abs",
                "start_position": [0, 0, 60],
                "runtime_jitter": [0, 0, 0],
            },
            "constraints": {"max_unit_cost": 1000.0, "max_weight_g": 5000.0},
            "benchmark_parts": [
                {
                    "part_id": "environment_fixture",
                    "label": "environment_fixture",
                    "metadata": {"fixed": True, "material_id": "aluminum_6061"},
                }
            ],
        }
    )
    return yaml.dump(benchmark.model_dump(mode="json"))


def _assembly_definition_yaml() -> str:
    cots_parts = [
        CotsPartEstimate(
            part_id="ServoMotor_DS3218",
            manufacturer="Generic",
            unit_cost_usd=18.0,
            weight_g=60.0,
            quantity=1,
            source="catalog",
        )
    ]

    assembly = AssemblyDefinition.model_validate(
        {
            "constraints": {
                "benchmark_max_unit_cost_usd": 1000.0,
                "benchmark_max_weight_g": 5000.0,
                "planner_target_max_unit_cost_usd": 900.0,
                "planner_target_max_weight_g": 4500.0,
            },
            "cots_parts": [part.model_dump(mode="json") for part in cots_parts],
            "final_assembly": [
                PartConfig(
                    name="drive_motor",
                    config=AssemblyPartConfig(dofs=[]),
                ).model_dump(mode="json")
            ],
            "totals": CostTotals(
                estimated_unit_cost_usd=18.0,
                estimated_weight_g=60.0,
                estimate_confidence="high",
            ).model_dump(mode="json"),
        }
    )
    return yaml.dump(assembly.model_dump(mode="json"))


def _solution_script_content() -> str:
    return """
from build123d import Compound
from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218", label="drive_motor")
    assembly = Compound(children=[motor], label="motor_assembly")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


def _missing_motor_script_content() -> str:
    return """
from build123d import Box, Compound
from shared.models.schemas import CompoundMetadata, PartMetadata


def build():
    body = Box(12, 12, 12)
    body.label = "placeholder_block"
    body.metadata = PartMetadata(material_id="aluminum_6061")
    assembly = Compound(children=[body], label="motor_assembly")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_129_cots_geometry_import_runtime_and_validation():
    """INT-129: COTS geometry import runtime and motor MVP."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)

        session_id = f"INT-129-{uuid.uuid4().hex[:8]}"

        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            _benchmark_definition_yaml(),
        )
        await _write_file(
            client,
            session_id,
            "assembly_definition.yaml",
            _assembly_definition_yaml(),
        )
        await _write_file(
            client,
            session_id,
            "solution_script.py",
            _solution_script_content(),
        )

        runtime_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "python - <<'PY'\n"
                    "import json\n"
                    "from solution_script import build\n"
                    "assembly = build()\n"
                    "motor = assembly.children[0]\n"
                    "bbox = motor.bounding_box()\n"
                    "payload = {\n"
                    "    'label': motor.label,\n"
                    "    'part_number': motor.part_number,\n"
                    "    'cots_id': motor.metadata.cots_id,\n"
                    "    'location': [\n"
                    "        motor.location.position.X,\n"
                    "        motor.location.position.Y,\n"
                    "        motor.location.position.Z,\n"
                    "    ],\n"
                    "    'bbox_min': [bbox.min.X, bbox.min.Y, bbox.min.Z],\n"
                    "    'bbox_max': [bbox.max.X, bbox.max.Y, bbox.max.Z],\n"
                    "}\n"
                    "print(json.dumps(payload))\n"
                    "PY"
                ),
                timeout=60,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert runtime_exec.status_code == 200, runtime_exec.text
        runtime_data = ExecuteResponse.model_validate(runtime_exec.json())
        assert runtime_data.exit_code == 0, runtime_data.stderr
        payload = json.loads(runtime_data.stdout.strip().splitlines()[-1])
        assert payload["label"] == "drive_motor"
        assert payload["part_number"] == "ServoMotor_DS3218"
        assert payload["cots_id"] == "ServoMotor_DS3218"
        assert payload["location"] == [0.0, 0.0, 0.0]
        assert payload["bbox_min"][2] == pytest.approx(0.0)
        assert payload["bbox_min"][0] < 0 < payload["bbox_max"][0]
        assert payload["bbox_max"][2] > 45.0

        invalid_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "python - <<'PY'\n"
                    "from shared.cots.parts.motors import ServoMotor\n"
                    "ServoMotor.from_catalog_id('ServoMotor_DOES_NOT_EXIST')\n"
                    "PY"
                ),
                timeout=30,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert invalid_exec.status_code == 200, invalid_exec.text
        invalid_data = ExecuteResponse.model_validate(invalid_exec.json())
        assert invalid_data.exit_code != 0
        assert (
            "Unknown catalog part_id 'ServoMotor_DOES_NOT_EXIST'" in invalid_data.stderr
        )

        validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
                backend=selected_backend(),
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success is True, validate_data.message
        assert validate_data.artifacts is not None
        assert validate_data.artifacts.simulation_result_json is not None

        await _write_file(
            client,
            session_id,
            "solution_script_missing_motor.py",
            _missing_motor_script_content(),
        )

        missing_validate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="solution_script_missing_motor.py",
                backend=selected_backend(),
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert missing_validate_resp.status_code == 200, missing_validate_resp.text
        missing_validate_data = BenchmarkToolResponse.model_validate(
            missing_validate_resp.json()
        )
        assert missing_validate_data.success is False
        assert missing_validate_data.artifacts is not None
        assert missing_validate_data.artifacts.failure is not None
        assert (
            missing_validate_data.artifacts.failure.reason
            == FailureReason.VALIDATION_FAILED
        )
        assert (
            "Declared COTS part(s) were not instantiated in authored geometry"
            in missing_validate_data.artifacts.failure.detail
        )
