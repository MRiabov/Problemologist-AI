import json
import os
import uuid

import httpx
import pytest
import yaml
from pydantic import BaseModel

from controller.api.schemas import CotsSearchItem
from shared.enums import ElectronicComponentType, FailureReason
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    AssemblyPartConfig,
    BenchmarkDefinition,
    CostTotals,
    CotsPartEstimate,
    ElectronicComponent,
    ElectronicsSection,
    MotorControl,
    PartConfig,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ElectronicsValidationRequest,
    ExecuteRequest,
    ExecuteResponse,
    VerificationRequest,
    WriteFileRequest,
)
from tests.integration.backend_utils import selected_backend

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")

pytestmark = pytest.mark.xdist_group(name="physics_sims")


def _default_benchmark_parts():
    return [
        {
            "part_id": "environment_fixture",
            "label": "environment_fixture",
            "metadata": {"fixed": True, "material_id": "aluminum_6061"},
        }
    ]


class PowerBudgetSummary(BaseModel):
    under_safe: bool
    under_margin_a: float
    over_safe: bool
    over_margin_a: float


class TransientSummary(BaseModel):
    ok: bool
    points: int
    duration_s: float
    sample_voltage: float | None = None


async def _require_services(client: httpx.AsyncClient) -> None:
    for name, url in (
        ("worker-light", WORKER_LIGHT_URL),
        ("worker-heavy", WORKER_HEAVY_URL),
        ("controller", CONTROLLER_URL),
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


def _base_benchmark_definition_yaml(
    backend: SimulatorBackendType | None = None,
) -> str:
    backend_value = (backend or selected_backend()).value
    objectives = BenchmarkDefinition.model_validate(
        {
            "physics": {"backend": backend_value},
            "objectives": {
                "goal_zone": {"min": [20, -10, 0], "max": [40, 10, 20]},
                "build_zone": {"min": [-100, -100, -10], "max": [100, 100, 100]},
            },
            "simulation_bounds": {"min": [-120, -120, -20], "max": [120, 120, 120]},
            "payload": {
                "label": "projectile_ball",
                "shape": "sphere",
                "material_id": "abs",
                "start_position": [0, 0, 60],
                "runtime_jitter": [0, 0, 0],
            },
            "constraints": {"max_unit_cost": 1000.0, "max_weight_g": 5000.0},
            "benchmark_parts": _default_benchmark_parts(),
        }
    )
    return yaml.dump(objectives.model_dump(mode="json"))


def _base_script() -> str:
    return """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    motor_1 = Box(8, 8, 8).move(Pos(-20, 0, 20))
    motor_1.label = "motor_1"
    motor_1.metadata = PartMetadata(material_id="aluminum_6061")

    rail = Box(80, 8, 8).move(Pos(0, 0, 4))
    rail.label = "rail"
    rail.metadata = PartMetadata(material_id="aluminum_6061")

    return Compound(children=[motor_1, rail], label="electromech")
"""


def _solution_motor_script() -> str:
    return """
from build123d import *
from shared.models.schemas import PartMetadata

def build():
    drive_motor = Box(8, 8, 8).move(Pos(-20, 0, 20))
    drive_motor.label = "drive_motor"
    drive_motor.metadata = PartMetadata(
        material_id="aluminum_6061",
        cots_id="ServoMotor_DS3218",
    )

    guide_rail = Box(80, 8, 8).move(Pos(0, 0, 4))
    guide_rail.label = "guide_rail"
    guide_rail.metadata = PartMetadata(material_id="aluminum_6061")

    return Compound(children=[drive_motor, guide_rail], label="solution_motor")
"""


def _solution_motor_assembly_definition_yaml() -> str:
    assembly = AssemblyDefinition(
        constraints=AssemblyConstraints(
            benchmark_max_unit_cost_usd=1000,
            benchmark_max_weight_g=5000,
            planner_target_max_unit_cost_usd=900,
            planner_target_max_weight_g=4500,
        ),
        cots_parts=[
            CotsPartEstimate(
                part_id="ServoMotor_DS3218",
                manufacturer="pololu",
                unit_cost_usd=18.0,
                weight_g=60.0,
                quantity=1,
                source="catalog",
            )
        ],
        manufactured_parts=[],
        final_assembly=[
            PartConfig(
                name="drive_motor",
                config=AssemblyPartConfig(
                    dofs=["rotate_z"],
                    control=MotorControl(mode="CONSTANT", speed=1.0),
                    cots_id="ServoMotor_DS3218",
                ),
            ),
            PartConfig(
                name="guide_rail",
                config=AssemblyPartConfig(dofs=[]),
            ),
        ],
        totals=CostTotals(
            estimated_unit_cost_usd=18.0,
            estimated_weight_g=60.0,
            estimate_confidence="high",
        ),
    )
    return yaml.dump(assembly.model_dump(mode="json"))


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_132_full_electromechanical_path():
    """
    INT-132: Full electromechanical workflow with split planning and unified implementation.
    This integration coverage validates the core mechanics + circuit + simulation path.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)
        session_id = f"INT-132-{uuid.uuid4().hex[:8]}"

        await _write_file(
            client,
            session_id,
            "benchmark_definition.yaml",
            _base_benchmark_definition_yaml(),
        )
        await _write_file(client, session_id, "script.py", _base_script())

        section = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
            components=[
                ElectronicComponent(
                    component_id="motor_1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=2.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="w_pos",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="motor_1", terminal="+"),
                    gauge_awg=18,
                    length_mm=120.0,
                ),
                WireConfig(
                    wire_id="w_neg",
                    from_terminal=WireTerminal(component="motor_1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=18,
                    length_mm=120.0,
                ),
            ],
        )
        assembly = AssemblyDefinition(
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=1000,
                benchmark_max_weight_g=5000,
                planner_target_max_unit_cost_usd=900,
                planner_target_max_weight_g=4500,
            ),
            electronics=section,
            totals=CostTotals(
                estimated_unit_cost_usd=120.0,
                estimated_weight_g=800.0,
                estimate_confidence="high",
            ),
        )
        await _write_file(
            client,
            session_id,
            "assembly_definition.yaml",
            yaml.dump(assembly.model_dump(mode="json")),
        )

        validate_circuit_resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=ElectronicsValidationRequest(section=section).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert validate_circuit_resp.status_code == 200, validate_circuit_resp.text
        validate_circuit_data = BenchmarkToolResponse.model_validate(
            validate_circuit_resp.json()
        )
        assert validate_circuit_data.success, validate_circuit_data.message
        assert (
            validate_circuit_data.artifacts is not None
            and validate_circuit_data.artifacts.circuit_validation_result is not None
        )
        assert (
            validate_circuit_data.artifacts.circuit_validation_result.get("valid")
            is True
        )

        validate_resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate",
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

        if (
            not simulate_data.success
            and simulate_data.artifacts
            and simulate_data.artifacts.failure
        ):
            assert simulate_data.artifacts.failure.reason not in {
                FailureReason.SHORT_CIRCUIT,
                FailureReason.OPEN_CIRCUIT,
                FailureReason.OVERCURRENT,
            }, simulate_data.artifacts.failure.detail or simulate_data.message

        verify_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/verify",
            json=VerificationRequest(
                script_path="script.py",
                smoke_test_mode=True,
                num_scenes=2,
                duration=0.5,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert verify_resp.status_code == 200, verify_resp.text
        verify_data = BenchmarkToolResponse.model_validate(verify_resp.json())
        assert verify_data.artifacts is not None
        assert verify_data.artifacts.verification_result is not None
        assert verify_data.artifacts.verification_result.num_scenes >= 1


@pytest.mark.int_id("INT-217")
@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_217_solution_motor_backend_parity():
    """
    INT-217: A solution-authored moving part should materialize as a validated
    controllable actuator on both supported backends, and unresolved mappings
    should fail closed before simulation can claim success.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)
        session_id = f"INT-217-{uuid.uuid4().hex[:8]}"

        await _write_file(
            client,
            session_id,
            "script.py",
            _solution_motor_script(),
        )

        for backend in (SimulatorBackendType.GENESIS, SimulatorBackendType.MUJOCO):
            await _write_file(
                client,
                session_id,
                "benchmark_definition.yaml",
                _base_benchmark_definition_yaml(backend=backend),
            )
            await _write_file(
                client,
                session_id,
                "assembly_definition.yaml",
                _solution_motor_assembly_definition_yaml(),
            )

            validate_resp = await client.post(
                f"{WORKER_LIGHT_URL}/benchmark/validate",
                json=BenchmarkToolRequest(script_path="script.py").model_dump(
                    mode="json"
                ),
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
            assert simulate_data.success, simulate_data.message

        await _write_file(
            client,
            session_id,
            "assembly_definition.yaml",
            _solution_motor_assembly_definition_yaml().replace(
                "ServoMotor_DS3218", "ServoMotor_DOES_NOT_EXIST"
            ),
        )
        fail_resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert fail_resp.status_code == 200, fail_resp.text
        fail_data = BenchmarkToolResponse.model_validate(fail_resp.json())
        assert not fail_data.success
        assert "supported COTS motor" in (fail_data.message or "")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_136_power_budget_validation():
    """INT-136: Power budget calculation validates under/over provisioning behavior."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)
        session_id = f"INT-136-{uuid.uuid4().hex[:8]}"

        code = """python - <<'PY'
import json
from shared.models.schemas import ElectronicComponent, ElectronicsSection, PowerSupplyConfig, WireConfig, WireTerminal
from shared.enums import ElectronicComponentType
from shared.pyspice_utils import calculate_static_power_budget

under = ElectronicsSection(
    power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
    components=[ElectronicComponent(component_id="m1", type=ElectronicComponentType.MOTOR, stall_current_a=3.0)],
    wiring=[
        WireConfig(wire_id="u1", **{"from": {"component":"supply","terminal":"v+"}, "to": {"component":"m1","terminal":"+"}}, gauge_awg=18, length_mm=100),
        WireConfig(wire_id="u2", **{"from": {"component":"m1","terminal":"-"}, "to": {"component":"supply","terminal":"0"}}, gauge_awg=18, length_mm=100),
    ],
)
over = ElectronicsSection(
    power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
    components=[ElectronicComponent(component_id="m1", type=ElectronicComponentType.MOTOR, stall_current_a=1.0)],
    wiring=[
        WireConfig(wire_id="o1", **{"from": {"component":"supply","terminal":"v+"}, "to": {"component":"m1","terminal":"+"}}, gauge_awg=18, length_mm=100),
        WireConfig(wire_id="o2", **{"from": {"component":"m1","terminal":"-"}, "to": {"component":"supply","terminal":"0"}}, gauge_awg=18, length_mm=100),
    ],
)
under_budget = calculate_static_power_budget(under)
over_budget = calculate_static_power_budget(over)
print(json.dumps({
    "under_safe": bool(under_budget["is_safe"]),
    "under_margin_a": float(under_budget["margin_a"]),
    "over_safe": bool(over_budget["is_safe"]),
    "over_margin_a": float(over_budget["margin_a"]),
}))
PY"""
        exec_resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(code=code, timeout=60).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=120.0,
        )
        assert exec_resp.status_code == 200, exec_resp.text
        payload = ExecuteResponse.model_validate(exec_resp.json())
        assert payload.exit_code == 0, payload.stderr
        summary = PowerBudgetSummary.model_validate(
            json.loads(payload.stdout.strip().splitlines()[-1])
        )

        assert summary.under_safe is False
        assert summary.under_margin_a < 0
        assert summary.over_safe is True
        assert summary.over_margin_a > 0


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_137_cots_electrical_component_search():
    """INT-137: COTS search yields electrical components with required fields."""
    async with httpx.AsyncClient(timeout=120.0) as client:
        await _require_services(client)

        all_results: list[CotsSearchItem] = []
        for query in ("power", "relay", "connector", "wire"):
            resp = await client.get(
                f"{CONTROLLER_URL}/api/cots/search", params={"q": query}
            )
            assert resp.status_code == 200, resp.text
            all_results.extend(
                CotsSearchItem.model_validate(item) for item in resp.json()
            )

        if not all_results:
            pytest.skip("COTS catalog returned no electrical search results.")

        categories = {item.category.lower() for item in all_results}
        assert any(
            key in " ".join(categories)
            for key in ("relay", "connector", "wire", "power", "elect")
        )
        for item in all_results:
            assert item.part_id
            assert item.name
            assert item.price >= 0
            assert item.source


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_140_wire_and_electrical_component_costing():
    """INT-140: validate_and_price path includes wire and electrical COTS costs."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_services(client)
        base_objectives = _base_benchmark_definition_yaml()
        base_script = _base_script()

        cots_part: CotsSearchItem | None = None
        for query in ("power", "relay", "connector", "wire"):
            resp = await client.get(
                f"{CONTROLLER_URL}/api/cots/search", params={"q": query}
            )
            assert resp.status_code == 200, resp.text
            results = [CotsSearchItem.model_validate(item) for item in resp.json()]
            if results:
                cots_part = sorted(results, key=lambda item: item.part_id)[0]
                break

        if cots_part is None:
            pytest.skip("COTS catalog returned no electrical search results.")

        assert cots_part.price > 0

        async def run_with_wire_length(length_mm: float) -> BenchmarkToolResponse:
            session_id = f"INT-140-{uuid.uuid4().hex[:8]}"
            await _write_file(
                client, session_id, "benchmark_definition.yaml", base_objectives
            )
            await _write_file(client, session_id, "script.py", base_script)

            assembly_yaml = yaml.safe_dump(
                {
                    "version": "1.0",
                    "constraints": {
                        "benchmark_max_unit_cost_usd": 1000,
                        "benchmark_max_weight_g": 5000,
                        "planner_target_max_unit_cost_usd": 950,
                        "planner_target_max_weight_g": 4500,
                    },
                    "cots_parts": [
                        {
                            "part_id": cots_part.part_id,
                            "manufacturer": cots_part.manufacturer,
                            "unit_cost_usd": cots_part.price,
                            "quantity": 1,
                            "source": cots_part.source,
                        }
                    ],
                    "electronics": {
                        "power_supply": {
                            "voltage_dc": 12.0,
                            "max_current_a": 10.0,
                        },
                        "components": [
                            {
                                "component_id": "motor_1",
                                "type": "MOTOR",
                                "stall_current_a": 1.0,
                            }
                        ],
                        "wiring": [
                            {
                                "wire_id": "w_pos",
                                "from": {
                                    "component": "supply",
                                    "terminal": "v+",
                                },
                                "to": {
                                    "component": "motor_1",
                                    "terminal": "+",
                                },
                                "gauge_awg": 18,
                                "length_mm": length_mm,
                            },
                            {
                                "wire_id": "w_neg",
                                "from": {
                                    "component": "motor_1",
                                    "terminal": "-",
                                },
                                "to": {
                                    "component": "supply",
                                    "terminal": "0",
                                },
                                "gauge_awg": 18,
                                "length_mm": length_mm,
                            },
                        ],
                    },
                    "totals": {
                        "estimated_unit_cost_usd": 150.0,
                        "estimated_weight_g": 800.0,
                        "estimate_confidence": "high",
                    },
                },
                sort_keys=False,
            )
            await _write_file(
                client, session_id, "assembly_definition.yaml", assembly_yaml
            )
            resp = await client.post(
                f"{WORKER_HEAVY_URL}/benchmark/simulate",
                json=BenchmarkToolRequest(
                    script_path="script.py", smoke_test_mode=True
                ).model_dump(mode="json"),
                headers={"X-Session-ID": session_id},
                timeout=300.0,
            )
            assert resp.status_code == 200, resp.text
            return BenchmarkToolResponse.model_validate(resp.json())

        short_wire = await run_with_wire_length(50.0)
        long_wire = await run_with_wire_length(5000.0)

        assert short_wire.artifacts is not None
        assert long_wire.artifacts is not None
        assert short_wire.artifacts.total_cost is not None
        assert long_wire.artifacts.total_cost is not None

        # Long wire path should increase computed total due to per-meter wire costing.
        assert long_wire.artifacts.total_cost > short_wire.artifacts.total_cost
        # COTS component cost should contribute regardless of length.
        assert short_wire.artifacts.total_cost >= cots_part.price


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_141_circuit_transient_simulation():
    """INT-141: simulate_circuit_transient returns time-series data for a valid netlist."""
    async with httpx.AsyncClient(timeout=180.0) as client:
        await _require_services(client)
        session_id = f"INT-141-{uuid.uuid4().hex[:8]}"

        code = """python - <<'PY'
import json
import numpy as np
from shared.models.schemas import ElectronicComponent, ElectronicsSection, PowerSupplyConfig, WireConfig, WireTerminal
from shared.enums import ElectronicComponentType
from shared.circuit_builder import build_circuit_from_section
from shared.pyspice_utils import simulate_circuit_transient

section = ElectronicsSection(
    power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=5.0),
    components=[ElectronicComponent(component_id="m1", type=ElectronicComponentType.MOTOR, stall_current_a=1.0)],
    wiring=[
        WireConfig(wire_id="t1", **{"from": {"component":"supply","terminal":"v+"}, "to": {"component":"m1","terminal":"+"}}, gauge_awg=20, length_mm=200),
        WireConfig(wire_id="t2", **{"from": {"component":"m1","terminal":"-"}, "to": {"component":"supply","terminal":"0"}}, gauge_awg=20, length_mm=200),
    ],
)
circuit = build_circuit_from_section(section)
tran = simulate_circuit_transient(circuit, duration_s=0.01, step_s=0.001)
time_points = np.array(tran.time)

sample_voltage = None
if len(tran.nodes):
    first_node = next(iter(tran.nodes.values()))
    sample_voltage = float(np.array(first_node)[0])

print(json.dumps({
    "ok": True,
    "points": int(len(time_points)),
    "duration_s": float(time_points[-1]) if len(time_points) else 0.0,
    "sample_voltage": sample_voltage,
}))
PY"""

        exec_resp = await client.post(
            f"{WORKER_LIGHT_URL}/runtime/execute",
            json=ExecuteRequest(code=code, timeout=60).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=120.0,
        )
        assert exec_resp.status_code == 200, exec_resp.text
        payload = ExecuteResponse.model_validate(exec_resp.json())
        assert payload.exit_code == 0, payload.stderr

        summary = TransientSummary.model_validate(
            json.loads(payload.stdout.strip().splitlines()[-1])
        )
        assert summary.ok is True
        assert summary.points >= 5
        assert summary.duration_s > 0
