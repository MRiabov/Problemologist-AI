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
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
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
        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
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
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
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
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
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
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
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
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
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
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json=sim_req.model_dump(mode="json"),
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert resp.status_code == 200
        data = BenchmarkToolResponse.model_validate(resp.json())
        assert data.success is True

        # 4. Verify totals in artifacts
        assert data.artifacts.total_cost is not None
        assert data.artifacts.total_cost > 0
        assert data.artifacts.total_weight_g is not None
        assert data.artifacts.total_weight_g > 0
