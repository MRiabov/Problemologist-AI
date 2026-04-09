import asyncio
import os
import uuid

import httpx
import pytest
import yaml

from shared.enums import ElectronicComponentType
from shared.models.schemas import (
    AssemblyConstraints,
    AssemblyDefinition,
    CostTotals,
    ElectronicComponent,
    ElectronicsSection,
    PowerSupplyConfig,
    WireConfig,
    WireTerminal,
)
from shared.workers.schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    ElectronicsValidationRequest,
    WriteFileRequest,
)
from tests.integration.backend_utils import skip_unless_genesis

pytestmark = pytest.mark.xdist_group(name="physics_sims")

# Constants
WORKER_LIGHT_URL = os.getenv(
    "WORKER_LIGHT_URL", os.getenv("WORKER_URL", "http://127.0.0.1:18001")
)
WORKER_HEAVY_URL = os.getenv(
    "WORKER_HEAVY_URL", os.getenv("WORKER_URL", "http://127.0.0.1:18001")
)


def get_session_id(tag: str) -> str:
    return f"INT-{tag}-{uuid.uuid4().hex[:8]}"


async def _require_worker_services(client: httpx.AsyncClient):
    for name, url in (
        ("worker-light", WORKER_LIGHT_URL),
        ("worker-heavy", WORKER_HEAVY_URL),
    ):
        try:
            resp = await client.get(f"{url}/health", timeout=5.0)
            resp.raise_for_status()
        except Exception:
            pytest.skip(f"{name} is not reachable at {url}")


async def _post_with_busy_retry(
    client: httpx.AsyncClient,
    *,
    url: str,
    json_payload: dict,
    headers: dict[str, str],
    timeout: float,
    max_attempts: int = 120,
) -> httpx.Response:
    """Retry transient heavy-worker busy responses during integration contention."""

    response: httpx.Response | None = None
    for _ in range(max_attempts):
        response = await client.post(
            url,
            json=json_payload,
            headers=headers,
            timeout=timeout,
        )
        if response.status_code != 503:
            return response

        try:
            ready_resp = await client.get(f"{WORKER_HEAVY_URL}/ready", timeout=5.0)
            if ready_resp.status_code == 200:
                continue
        except Exception:
            pass

        await asyncio.sleep(1.0)

    assert response is not None
    return response


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_120_circuit_validation_gate():
    """INT-120: Verify that /validate_circuit endpoint works and simulate respects its failure."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("120")
        # 1. Define a faulty circuit (short circuit)
        faulty_section = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
            components=[],
            wiring=[
                WireConfig(
                    wire_id="short_wire",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=10,
                    length_mm=10000.0,  # ~0.033 Ohm -> ~360A
                )
            ],
        )

        # 2. Call /benchmark/validate_circuit
        req_val = ElectronicsValidationRequest(section=faulty_section)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=req_val.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        val_resp = BenchmarkToolResponse.model_validate(resp.json())
        assert val_resp.success is False
        assert val_resp.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "SHORT_CIRCUIT" in val_resp.artifacts.failure.detail

        # 3. Try to run simulation with this faulty circuit in assembly_definition.yaml
        # First write assembly_definition.yaml
        assembly_def = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=100,
                benchmark_max_weight_g=1000,
                planner_target_max_unit_cost_usd=80,
                planner_target_max_weight_g=800,
            ),
            electronics=faulty_section,
            totals=CostTotals(
                estimated_unit_cost_usd=50,
                estimated_weight_g=500,
                estimate_confidence="high",
            ),
        )

        req_write_asm = WriteFileRequest(
            path="assembly_definition.yaml",
            content=yaml.dump(assembly_def.model_dump(mode="json")),
            overwrite=True,
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_asm.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # Write a dummy script.py
        script = """
from build123d import *
def build():
    p = Box(1, 1, 1)
    p.label = "test_box"
    from shared.models.schemas import PartMetadata
    p.metadata = PartMetadata(material_id='aluminum_6061')
    return p
"""
        req_write_script = WriteFileRequest(
            path="script.py", content=script, overwrite=True
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=req_write_script.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # 4. Trigger simulation on worker_heavy
        sim_req = BenchmarkToolRequest(script_path="script.py")
        sim_resp = await _post_with_busy_retry(
            client,
            url=f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json_payload=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert sim_resp.status_code == 200
        sim_data = BenchmarkToolResponse.model_validate(sim_resp.json())
        assert sim_data.success is False
        assert sim_data.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "SHORT_CIRCUIT" in sim_data.artifacts.failure.detail


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_121_short_circuit_detection():
    """INT-121: Verify short circuit detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("121")
        short_circuit = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
            components=[],
            wiring=[
                WireConfig(
                    wire_id="short_wire_121",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=10,
                    length_mm=10000.0,
                )
            ],
        )
        req_val = ElectronicsValidationRequest(section=short_circuit)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=req_val.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        val_resp = BenchmarkToolResponse.model_validate(resp.json())
        assert val_resp.success is False
        assert val_resp.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "SHORT_CIRCUIT" in val_resp.artifacts.failure.detail


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_122_overcurrent_supply_detection():
    """INT-122: Verify overcurrent supply detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("122")
        # Motor with 5A stall on a 1A PSU. Resistance = 12/5 = 2.4 Ohm.
        overcurrent = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=1.0),
            components=[
                ElectronicComponent(
                    component_id="m1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=5.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="w1",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=18,
                    length_mm=500.0,
                ),
                WireConfig(
                    wire_id="w2",
                    from_terminal=WireTerminal(component="m1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=18,
                    length_mm=500.0,
                ),
            ],
        )
        req_val = ElectronicsValidationRequest(section=overcurrent)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=req_val.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        val_resp = BenchmarkToolResponse.model_validate(resp.json())
        assert val_resp.success is False
        assert val_resp.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "OVERCURRENT" in val_resp.artifacts.failure.detail


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_123_overcurrent_wire_detection():
    """INT-123: Verify overcurrent wire detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("123")
        # High current (10A) on a very thin wire (AWG 30)
        # AWG 30 rated for ~0.5A
        wire_overcurrent = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=20.0),
            components=[
                ElectronicComponent(
                    component_id="m1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=10.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="thin_wire",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=30,
                    length_mm=10.0,
                ),
                WireConfig(
                    wire_id="w2",
                    from_terminal=WireTerminal(component="m1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=18,
                    length_mm=10.0,
                ),
            ],
        )
        req_val = ElectronicsValidationRequest(section=wire_overcurrent)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=req_val.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        val_resp = BenchmarkToolResponse.model_validate(resp.json())
        assert val_resp.success is False
        assert val_resp.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "OVERCURRENT" in val_resp.artifacts.failure.detail


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_124_open_circuit_detection():
    """INT-124: Verify open circuit / floating node detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("124")
        # Motor with only one terminal connected
        open_circuit = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
            components=[
                ElectronicComponent(
                    component_id="m1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=2.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="w1",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=22,
                    length_mm=100.0,
                )
                # Missing wire from m1:- to supply:0
            ],
        )
        req_val = ElectronicsValidationRequest(section=open_circuit)
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate_circuit",
            json=req_val.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        val_resp = BenchmarkToolResponse.model_validate(resp.json())
        assert val_resp.success is False
        assert val_resp.artifacts.failure.reason == "VALIDATION_FAILED"
        assert "OPEN_CIRCUIT" in val_resp.artifacts.failure.detail


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_125_valid_circuit_totals():
    """INT-125: Verify valid circuit returns cost and weight artifacts."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("125")

        # 1. Define a valid circuit (Motor on 12V PSU)
        valid_section = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
            components=[
                ElectronicComponent(
                    component_id="m1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=2.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="w1",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=18,
                    length_mm=100.0,
                ),
                WireConfig(
                    wire_id="w2",
                    from_terminal=WireTerminal(component="m1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=18,
                    length_mm=100.0,
                ),
            ],
        )

        # 2. Setup assembly definition
        assembly_def = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=100,
                benchmark_max_weight_g=1000,
                planner_target_max_unit_cost_usd=80,
                planner_target_max_weight_g=800,
            ),
            electronics=valid_section,
            totals=CostTotals(
                estimated_unit_cost_usd=50,
                estimated_weight_g=500,
                estimate_confidence="high",
            ),
        )

        # Write assembly_definition.yaml
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="assembly_definition.yaml",
                content=yaml.dump(assembly_def.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # Write script.py
        script = """
from build123d import *
def build():
    p = Box(1, 1, 1)
    p.label = "test_box"
    from shared.models.schemas import PartMetadata
    p.metadata = PartMetadata(material_id='aluminum_6061')
    return p
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py", content=script, overwrite=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        # 3. Simulate
        sim_req = BenchmarkToolRequest(script_path="script.py", smoke_test_mode=True)
        resp = await _post_with_busy_retry(
            client,
            url=f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json_payload=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        if not data.success:
            pytest.fail(f"Simulation failed: {data.message}")

        # 4. Verify totals in artifacts
        assert data.artifacts.total_cost is not None
        assert data.artifacts.total_cost > 0
        assert data.artifacts.total_weight_g is not None
        assert data.artifacts.total_weight_g > 0


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_126_wire_tear_behavior():
    """
    INT-126: Simulation fails with FAILED_WIRE_TORN
    when wire tension exceeds tensile rating.
    """
    skip_unless_genesis(
        "INT-126 requires backend-supported dynamic wire/softbody behavior"
    )
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("126")

        electronics = ElectronicsSection(
            power_supply=PowerSupplyConfig(voltage_dc=12.0, max_current_a=10.0),
            components=[
                ElectronicComponent(
                    component_id="m1",
                    type=ElectronicComponentType.MOTOR,
                    stall_current_a=1.0,
                )
            ],
            wiring=[
                WireConfig(
                    wire_id="wire_torn_test",
                    from_terminal=WireTerminal(component="supply", terminal="v+"),
                    to_terminal=WireTerminal(component="m1", terminal="+"),
                    gauge_awg=40,
                    length_mm=50.0,
                    routed_in_3d=True,
                    waypoints=[(0.0, 0.0, 20.0), (500.0, 0.0, 20.0)],
                ),
                WireConfig(
                    wire_id="wire_return",
                    from_terminal=WireTerminal(component="m1", terminal="-"),
                    to_terminal=WireTerminal(component="supply", terminal="0"),
                    gauge_awg=40,
                    length_mm=50.0,
                    routed_in_3d=True,
                    waypoints=[(0.0, 10.0, 20.0), (500.0, 10.0, 20.0)],
                ),
            ],
        )
        assembly_def = AssemblyDefinition(
            version="1.0",
            constraints=AssemblyConstraints(
                benchmark_max_unit_cost_usd=100.0,
                benchmark_max_weight_g=1000.0,
                planner_target_max_unit_cost_usd=80.0,
                planner_target_max_weight_g=800.0,
            ),
            electronics=electronics,
            totals=CostTotals(
                estimated_unit_cost_usd=10.0,
                estimated_weight_g=100.0,
                estimate_confidence="high",
            ),
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="assembly_definition.yaml",
                content=yaml.dump(assembly_def.model_dump(mode="json")),
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(5, 5, 5).move(Location((0, 0, 10)))
    p.label = "target_box"
    p.metadata = PartMetadata(material_id='aluminum_6061')
    return p
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py", content=script, overwrite=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert data.artifacts.failure is not None
        assert str(data.artifacts.failure.reason).endswith("WIRE_TORN")
        assert "wire_torn_test" in (data.artifacts.failure.detail or "")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_128_objectives_electronics_schema_gate():
    """
    INT-128: malformed electronics_requirements in benchmark_definition.yaml
    fails closed in /benchmark/validate.
    """
    async with httpx.AsyncClient(timeout=300.0) as client:
        await _require_worker_services(client)
        session_id = get_session_id("128")
        script = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    p = Box(1, 1, 1)
    p.label = "target_box"
    p.metadata = PartMetadata(material_id='aluminum_6061')
    return p
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="script.py", content=script, overwrite=True
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )
        invalid_objectives = """
objectives:
  goal_zone: {min: [0, 0, 0], max: [1, 1, 1]}
  forbid_zones: []
  build_zone: {min: [-10, -10, -10], max: [10, 10, 10]}
simulation_bounds: {min: [-20, -20, -20], max: [20, 20, 20]}
payload:
  label: target_box
  shape: sphere
  material_id: abs
  start_position: [0, 0, 0]
  runtime_jitter: [0, 0, 0]
constraints:
  max_unit_cost: 100
  max_weight_g: 1000
benchmark_parts:
  - part_id: environment_fixture
    label: environment_fixture
    metadata:
      fixed: true
      material_id: aluminum_6061
electronics_requirements:
  power_supply_available:
    voltage_dc: 12
  wiring_constraints: bad
  circuit_validation_required: "sometimes"
"""
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json=WriteFileRequest(
                path="benchmark_definition.yaml",
                content=invalid_objectives,
                overwrite=True,
            ).model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
        )

        resp = await client.post(
            f"{WORKER_LIGHT_URL}/benchmark/validate",
            json=BenchmarkToolRequest(script_path="script.py").model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=180.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is False
        assert "electronics_requirements" in (data.message or "")
