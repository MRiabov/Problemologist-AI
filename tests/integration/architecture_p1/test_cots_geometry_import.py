import hashlib
import json
import uuid

import httpx
import pytest
import yaml

from shared.enums import AgentName, FailureReason
from shared.models.schemas import (
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    CostTotals,
    CotsPartEstimate,
    DraftingCallout,
    DraftingDimension,
    DraftingSheet,
    DraftingView,
    PartConfig,
)
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ExecuteRequest,
    ExecuteResponse,
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
    WriteFileRequest,
)
from tests.integration.backend_utils import selected_backend
from worker_heavy.utils.file_validation import (
    _assembly_script_expected_tokens,
    _benchmark_script_expected_tokens,
    _validate_assembly_inventory_parity,
)

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
            "payload": {
                "label": "payload",
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
                    config=AssemblyPartConfig(
                        dofs=[],
                        cots_id="ServoMotor_DS3218",
                    ),
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
    body.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    assembly = Compound(children=[body], label="motor_assembly")
    assembly.metadata = CompoundMetadata(fixed=True)
    return assembly
"""


def _wrong_label_motor_script_content() -> str:
    return """
from build123d import Compound
from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    motor = ServoMotor.from_catalog_id("ServoMotor_DS3218", label="wrong_motor")
    assembly = Compound(children=[motor], label="motor_assembly")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


def _paired_motor_script_content() -> str:
    return """
from build123d import Compound, Location
from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    left_motor = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218", label="left_motor"
    ).move(Location((-60.0, 0.0, 0.0)))
    right_motor = ServoMotor.from_catalog_id(
        "ServoMotor_MG996R", label="right_motor"
    ).move(Location((60.0, 0.0, 0.0)))
    assembly = Compound(children=[left_motor, right_motor], label="motor_pair")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


def _pair_swapped_motor_script_content() -> str:
    return """
from build123d import Compound, Location
from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    left_motor = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218", label="right_motor"
    ).move(Location((-60.0, 0.0, 0.0)))
    right_motor = ServoMotor.from_catalog_id(
        "ServoMotor_MG996R", label="left_motor"
    ).move(Location((60.0, 0.0, 0.0)))
    assembly = Compound(children=[left_motor, right_motor], label="motor_pair")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


def _paired_motor_technical_drawing_script_content() -> str:
    return """
from build123d import Compound, Location, TechnicalDrawing
from shared.cots.parts.motors import ServoMotor
from shared.models.schemas import CompoundMetadata


def build():
    TechnicalDrawing(title="Seeded drafting")
    left_motor = ServoMotor.from_catalog_id(
        "ServoMotor_DS3218", label="left_motor"
    ).move(Location((-60.0, 0.0, 0.0)))
    right_motor = ServoMotor.from_catalog_id(
        "ServoMotor_MG996R", label="right_motor"
    ).move(Location((60.0, 0.0, 0.0)))
    assembly = Compound(children=[left_motor, right_motor], label="motor_pair")
    assembly.metadata = CompoundMetadata()
    return assembly
"""


async def _seed_engineer_drafting_render_manifest(
    client: httpx.AsyncClient,
    session_id: str,
    *,
    technical_drawing_script_content: str,
) -> None:
    preview_path = "renders/engineer_plan_renders/render_pair.png"
    svg_path = "renders/engineer_plan_renders/render_pair.svg"
    dxf_path = "renders/engineer_plan_renders/render_pair.dxf"
    manifest = RenderManifest(
        episode_id=session_id,
        worker_session_id=session_id,
        bundle_path="renders/engineer_plan_renders",
        drafting=True,
        source_script_sha256=hashlib.sha256(
            technical_drawing_script_content.encode("utf-8")
        ).hexdigest(),
        preview_evidence_paths=[preview_path],
        artifacts={
            preview_path: RenderArtifactMetadata(
                modality="rgb",
                siblings=RenderSiblingPaths(
                    rgb=preview_path,
                    svg=svg_path,
                    dxf=dxf_path,
                ),
            )
        },
    )
    manifest_json = manifest.model_dump_json(indent=2)
    code = f"""
python - <<'PY'
from pathlib import Path
import base64

root = Path("renders/engineer_plan_renders")
root.mkdir(parents=True, exist_ok=True)
(root / "render_pair.png").write_bytes(
    base64.b64decode("iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5yM1kAAAAASUVORK5CYII=")
)
(root / "render_pair.svg").write_text(
    "<svg xmlns='http://www.w3.org/2000/svg' width='1' height='1'></svg>",
    encoding="utf-8",
)
(root / "render_pair.dxf").write_text(
    "0\\nSECTION\\n2\\nENTITIES\\n0\\nENDSEC\\n0\\nEOF\\n",
    encoding="utf-8",
)
(root / "render_manifest.json").write_text({manifest_json!r}, encoding="utf-8")
PY
"""
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/runtime/execute",
        json=ExecuteRequest(code=code, timeout=60).model_dump(mode="json"),
        headers={"X-Session-ID": session_id},
    )
    assert resp.status_code == 200, resp.text
    data = ExecuteResponse.model_validate(resp.json())
    assert data.exit_code == 0, data.stderr


def _paired_motor_assembly_definition_yaml() -> str:
    assembly = AssemblyDefinition.model_validate(
        {
            "constraints": {
                "benchmark_max_unit_cost_usd": 1000.0,
                "benchmark_max_weight_g": 5000.0,
                "planner_target_max_unit_cost_usd": 900.0,
                "planner_target_max_weight_g": 4500.0,
            },
            "cots_parts": [
                {
                    "part_id": "ServoMotor_DS3218",
                    "manufacturer": "Generic",
                    "unit_cost_usd": 18.0,
                    "weight_g": 60.0,
                    "quantity": 1,
                    "source": "catalog",
                },
                {
                    "part_id": "ServoMotor_MG996R",
                    "manufacturer": "Generic",
                    "unit_cost_usd": 12.0,
                    "weight_g": 55.0,
                    "quantity": 1,
                    "source": "catalog",
                },
            ],
            "final_assembly": [
                {
                    "name": "left_motor",
                    "config": {
                        "dofs": [],
                        "cots_id": "ServoMotor_DS3218",
                    },
                },
                {
                    "name": "right_motor",
                    "config": {
                        "dofs": [],
                        "cots_id": "ServoMotor_MG996R",
                    },
                },
            ],
            "totals": {
                "estimated_unit_cost_usd": 30.0,
                "estimated_weight_g": 115.0,
                "estimate_confidence": "high",
            },
            "drafting": DraftingSheet(
                sheet_id="sheet-1",
                title="Motor Pair Drafting",
                views=[
                    DraftingView(
                        view_id="front",
                        target="left_motor",
                        projection="front",
                        scale=1.0,
                        datums=["A", "B"],
                        dimensions=[
                            DraftingDimension(
                                dimension_id="motor_spacing",
                                kind="linear",
                                target="left_motor",
                                value=120.0,
                                binding=True,
                            )
                        ],
                        callouts=[
                            DraftingCallout(
                                callout_id="1",
                                label="left_motor",
                                target="left_motor",
                            ),
                            DraftingCallout(
                                callout_id="2",
                                label="right_motor",
                                target="right_motor",
                            ),
                        ],
                    )
                ],
            ).model_dump(mode="json"),
        }
    )
    return yaml.dump(assembly.model_dump(mode="json"))


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
            "plan.md",
            """# Engineering Plan

## 1. Solution Overview
Use a catalog-backed `drive_motor` to actuate the gate.

## 2. Parts List
| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| drive_motor | catalog `ServoMotor_DS3218` | cots | Primary actuator for the gate |

## 3. Assembly Strategy
1. Mount `drive_motor` at the rear-left side and keep the cable corridor clear.

## 4. Assumption Register
- Assumption: The planner relies on source-backed inputs that must be traceable.

## 5. Detailed Calculations
| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Example calculation supporting the plan | `N/A` | Replace this placeholder with the actual derived limit. |

### CALC-001: Example calculation supporting the plan

#### Problem Statement

The plan needs a traceable calculation instead of a freeform claim.

#### Assumptions

- `ASSUMP-001`: The input values are taken from the benchmark or assembly definition.

#### Derivation

- Compute the binding quantity from the declared inputs.

#### Worst-Case Check

- The derived limit must hold under the worst-case allowed inputs.

#### Result

- The design remains valid only if the derived limit is respected.

#### Design Impact

- Update the design or inputs if the calculation changes.

#### Cross-References

- `plan.md#3-assembly-strategy`

## 6. Critical Constraints / Operating Envelope
- Constraint: The mechanism must remain inside the derived operating limits.

## 7. Cost & Weight Budget
- Keep the solution inside the seeded cost and weight bounds.

## 8. Risk Assessment
- Avoid routing the motor cable through the sweep path.
""",
        )
        await _write_file(
            client,
            session_id,
            "todo.md",
            """# Engineering Checklist

- [x] Confirm the catalog-backed motor selection.
- [x] Preserve the rear-left cable corridor.
- [x] Re-run validation and simulation before submission.
""",
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

        placement_exec = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(
                code=(
                    "python - <<'PY'\n"
                    "import json\n"
                    "from build123d import Location\n"
                    "from shared.cots.parts.motors import ServoMotor\n"
                    "motor = ServoMotor.from_catalog_id('ServoMotor_DS3218', label='drive_motor')\n"
                    "motor = motor.move(Location((12.5, -3.0, 7.5), (0, 0, 90)))\n"
                    "payload = {\n"
                    "    'location': [\n"
                    "        motor.location.position.X,\n"
                    "        motor.location.position.Y,\n"
                    "        motor.location.position.Z,\n"
                    "    ],\n"
                    "    'orientation': [\n"
                    "        motor.location.orientation.X,\n"
                    "        motor.location.orientation.Y,\n"
                    "        motor.location.orientation.Z,\n"
                    "    ],\n"
                    "}\n"
                    "print(json.dumps(payload))\n"
                    "PY"
                ),
                timeout=60,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert placement_exec.status_code == 200, placement_exec.text
        placement_data = ExecuteResponse.model_validate(placement_exec.json())
        assert placement_data.exit_code == 0, placement_data.stderr
        placement_payload = json.loads(placement_data.stdout.strip().splitlines()[-1])
        assert placement_payload["location"] == [12.5, -3.0, 7.5]
        assert placement_payload["orientation"][2] == pytest.approx(90.0)

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
            f"{WORKER_LIGHT_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success is True, validate_data.message

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

        await _write_file(
            client,
            session_id,
            "solution_script.py",
            _wrong_label_motor_script_content(),
        )

        wrong_label_validate_resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert wrong_label_validate_resp.status_code == 200, (
            wrong_label_validate_resp.text
        )
        wrong_label_validate_data = BenchmarkToolResponse.model_validate(
            wrong_label_validate_resp.json()
        )
        assert wrong_label_validate_data.success is True, (
            wrong_label_validate_data.message
        )

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success is False
        assert submit_data.message is not None
        assert "exact inventory mismatch for 'drive_motor'" in submit_data.message
        assert "observed identities: label=wrong_motor, cots_id=ServoMotor_DS3218" in (
            submit_data.message
        )


def test_int_130_inventory_expected_tokens_do_not_double_count_final_assembly():
    benchmark_definition = BenchmarkDefinition.model_validate(
        {
            "objectives": {
                "goal_zone": {"min": [20.0, -10.0, 0.0], "max": [40.0, 10.0, 20.0]},
                "forbid_zones": [],
                "build_zone": {
                    "min": [-140.0, -140.0, -20.0],
                    "max": [140.0, 140.0, 140.0],
                },
            },
            "benchmark_parts": [
                {
                    "part_id": "fixture_a",
                    "label": "fixture_a",
                    "metadata": {"fixed": True, "material_id": "aluminum_6061"},
                },
                {
                    "part_id": "fixture_b",
                    "label": "fixture_b",
                    "metadata": {"fixed": True, "material_id": "hdpe"},
                },
            ],
            "physics": {"backend": "GENESIS"},
            "fluids": [],
            "simulation_bounds": {
                "min": [-200.0, -200.0, -50.0],
                "max": [200.0, 200.0, 200.0],
            },
            "payload": {
                "label": "target_ball",
                "shape": "sphere",
                "material_id": "abs",
                "start_position": [0.0, 0.0, 50.0],
                "runtime_jitter": [0.0, 0.0, 0.0],
            },
            "constraints": {"max_unit_cost": 100.0, "max_weight_g": 1000.0},
        }
    )
    benchmark_assembly = AssemblyDefinition.model_validate(
        {
            "constraints": {
                "planner_target_max_unit_cost_usd": 90.0,
                "planner_target_max_weight_g": 900.0,
            },
            "manufactured_parts": [
                {
                    "part_name": "fixture_a",
                    "part_id": "fixture_a",
                    "manufacturing_method": "CNC",
                    "material_id": "aluminum_6061",
                    "quantity": 1,
                    "part_volume_mm3": 1000.0,
                    "stock_bbox_mm": {"x": 10.0, "y": 10.0, "z": 10.0},
                    "stock_volume_mm3": 1000.0,
                    "removed_volume_mm3": 0.0,
                    "estimated_unit_cost_usd": 5.0,
                },
                {
                    "part_name": "fixture_b",
                    "part_id": "fixture_b",
                    "manufacturing_method": "CNC",
                    "material_id": "hdpe",
                    "quantity": 1,
                    "part_volume_mm3": 1000.0,
                    "stock_bbox_mm": {"x": 10.0, "y": 10.0, "z": 10.0},
                    "stock_volume_mm3": 1000.0,
                    "removed_volume_mm3": 0.0,
                    "estimated_unit_cost_usd": 5.0,
                },
            ],
            "cots_parts": [],
            "final_assembly": [
                {
                    "name": "fixture_a",
                    "config": {"dofs": [], "cots_id": None},
                },
                {
                    "name": "fixture_b",
                    "config": {"dofs": [], "cots_id": None},
                },
            ],
            "totals": {
                "estimated_unit_cost_usd": 5.0,
                "estimated_weight_g": 10.0,
                "estimate_confidence": "high",
            },
        }
    )
    assert _validate_assembly_inventory_parity(benchmark_assembly) == []
    benchmark_tokens = _benchmark_script_expected_tokens(
        benchmark_definition=benchmark_definition,
        assembly_definition=benchmark_assembly,
    )
    assert benchmark_tokens["fixture_a"] == 1
    assert benchmark_tokens["fixture_b"] == 1

    benchmark_assembly_mismatch = AssemblyDefinition.model_validate(
        {
            "constraints": {
                "planner_target_max_unit_cost_usd": 90.0,
                "planner_target_max_weight_g": 900.0,
            },
            "manufactured_parts": [
                {
                    "part_name": "fixture_a",
                    "part_id": "fixture_a",
                    "manufacturing_method": "CNC",
                    "material_id": "aluminum_6061",
                    "quantity": 1,
                    "part_volume_mm3": 1000.0,
                    "stock_bbox_mm": {"x": 10.0, "y": 10.0, "z": 10.0},
                    "stock_volume_mm3": 1000.0,
                    "removed_volume_mm3": 0.0,
                    "estimated_unit_cost_usd": 5.0,
                }
            ],
            "cots_parts": [],
            "final_assembly": [
                {
                    "name": "fixture_a",
                    "config": {"dofs": [], "cots_id": None},
                },
                {
                    "name": "fixture_a",
                    "config": {"dofs": [], "cots_id": None},
                },
            ],
            "totals": {
                "estimated_unit_cost_usd": 5.0,
                "estimated_weight_g": 10.0,
                "estimate_confidence": "high",
            },
        }
    )
    parity_errors = _validate_assembly_inventory_parity(benchmark_assembly_mismatch)
    assert any(
        "final_assembly parity mismatch for 'fixture_a'" in error
        for error in parity_errors
    )

    engineer_assembly = AssemblyDefinition.model_validate(
        {
            "constraints": {
                "planner_target_max_unit_cost_usd": 90.0,
                "planner_target_max_weight_g": 900.0,
            },
            "manufactured_parts": [
                {
                    "part_name": "base_plate",
                    "part_id": "base_plate",
                    "manufacturing_method": "CNC",
                    "material_id": "aluminum_6061",
                    "quantity": 1,
                    "part_volume_mm3": 1000.0,
                    "stock_bbox_mm": {"x": 10.0, "y": 10.0, "z": 10.0},
                    "stock_volume_mm3": 1000.0,
                    "removed_volume_mm3": 0.0,
                    "estimated_unit_cost_usd": 5.0,
                }
            ],
            "cots_parts": [
                {
                    "part_id": "ServoMotor_DS3218",
                    "manufacturer": "pololu",
                    "unit_cost_usd": 18.0,
                    "weight_g": 60.0,
                    "quantity": 2,
                    "source": "catalog",
                }
            ],
            "final_assembly": [
                {"name": "base_plate", "config": {"dofs": []}},
                {
                    "name": "drive_motor",
                    "config": {"dofs": [], "cots_id": "ServoMotor_DS3218"},
                },
                {
                    "name": "drive_motor",
                    "config": {"dofs": [], "cots_id": "ServoMotor_DS3218"},
                },
            ],
            "totals": {
                "estimated_unit_cost_usd": 41.0,
                "estimated_weight_g": 130.0,
                "estimate_confidence": "high",
            },
        }
    )
    engineer_tokens = _assembly_script_expected_tokens(engineer_assembly)
    assert engineer_tokens["base_plate"] == 1
    assert engineer_tokens["drive_motor"] == 2
    assert engineer_tokens["ServoMotor_DS3218"] == 2


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_131_pair_swapped_cots_rows_fail_inventory_exactness():
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)
        session_id = f"INT-131-{uuid.uuid4().hex[:8]}"

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
            _paired_motor_assembly_definition_yaml(),
        )
        await _write_file(
            client,
            session_id,
            "plan.md",
            """# Engineering Plan

## 1. Solution Overview
Use a two-motor assembly with `left_motor` and `right_motor`.

## 2. Parts List
| Part | Dimensions (mm) | Material | Purpose |
| -- | -- | -- | -- |
| left_motor | catalog `ServoMotor_DS3218` | cots | Left actuator |
| right_motor | catalog `ServoMotor_MG996R` | cots | Right actuator |

## 3. Assembly Strategy
1. Mount `left_motor` on the left side and `right_motor` on the right side.

## 4. Assumption Register
- Assumption: The selected catalog motors are the source-backed inputs.

## 5. Detailed Calculations
| ID | Problem / Decision | Result | Impact |
| -- | -- | -- | -- |
| CALC-001 | Example calculation supporting the plan | `N/A` | Replace this placeholder with the actual derived limit. |

### CALC-001: Example calculation supporting the plan

#### Problem Statement

The plan needs a traceable calculation instead of a freeform claim.

#### Assumptions

- `ASSUMP-001`: The input values are taken from the benchmark or assembly definition.

#### Derivation

- Compute the binding quantity from the declared inputs.

#### Worst-Case Check

- The derived limit must hold under the worst-case allowed inputs.

#### Result

- The design remains valid only if the derived limit is respected.

#### Design Impact

- Update the design or inputs if the calculation changes.

#### Cross-References

- `plan.md#3-assembly-strategy`

## 6. Critical Constraints / Operating Envelope
- Constraint: The mechanism must remain inside the derived operating limits.

## 7. Cost & Weight Budget
- Keep the solution inside the seeded cost and weight bounds.

## 8. Risk Assessment
- Avoid swapping the left and right catalog identities.
- Reference intent: the technical drawing script is a reference artifact for the goal zone.
""",
        )
        await _write_file(
            client,
            session_id,
            "todo.md",
            """# Engineering Checklist

- [x] Confirm the left/right catalog pairing.
- [x] Preserve the declared identity pairs in authored geometry.
- [x] Re-run validation and simulation before submission.
""",
        )
        await _write_file(
            client,
            session_id,
            "solution_script.py",
            _paired_motor_script_content(),
        )
        await _write_file(
            client,
            session_id,
            "solution_plan_technical_drawing_script.py",
            _paired_motor_technical_drawing_script_content(),
        )
        await _seed_engineer_drafting_render_manifest(
            client,
            session_id,
            technical_drawing_script_content=_paired_motor_technical_drawing_script_content(),
        )

        validate_resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="solution_script.py").model_dump(
                mode="json"
            ),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_resp.status_code == 200, validate_resp.text
        validate_data = BenchmarkToolResponse.model_validate(validate_resp.json())
        assert validate_data.success is True, validate_data.message

        simulate_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
                backend=selected_backend(),
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert simulate_resp.status_code == 200, simulate_resp.text
        simulate_data = BenchmarkToolResponse.model_validate(simulate_resp.json())
        assert simulate_data.success is True, simulate_data.message

        submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert submit_resp.status_code == 200, submit_resp.text
        submit_data = BenchmarkToolResponse.model_validate(submit_resp.json())
        assert submit_data.success is True, submit_data.message

        await _write_file(
            client,
            session_id,
            "solution_script.py",
            _pair_swapped_motor_script_content(),
        )

        swapped_submit_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/submit",
            json=BenchmarkToolRequest(
                script_path="solution_script.py",
                reviewer_stage=AgentName.ENGINEER_EXECUTION_REVIEWER,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert swapped_submit_resp.status_code == 200, swapped_submit_resp.text
        swapped_submit_data = BenchmarkToolResponse.model_validate(
            swapped_submit_resp.json()
        )
        assert swapped_submit_data.success is False
        assert swapped_submit_data.message is not None
        assert "exact inventory pair mismatch" in swapped_submit_data.message
