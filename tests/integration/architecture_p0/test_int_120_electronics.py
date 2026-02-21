import os
import time
import pytest
import httpx

import uuid

# Constants
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


def get_session_id(tag: str) -> str:
    return f"INT-{tag}-{uuid.uuid4().hex[:8]}"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_120_circuit_validation_gate():
    """INT-120: Verify that /validate_circuit endpoint works and simulate respects its failure."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = get_session_id("120")
        # 1. Define a faulty circuit (short circuit)
        faulty_section = {
            "power_supply": {"voltage_dc": 12.0, "max_current_a": 10.0},
            "components": [],
            "wiring": [
                {
                    "wire_id": "short_wire",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "supply", "terminal": "0"},
                    "gauge_awg": 10,
                    "length_mm": 10000.0,  # ~0.033 Ohm -> ~360A
                }
            ],
        }

        # 2. Call /benchmark/validate_circuit
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
            json={"section": faulty_section},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["success"] is False
        assert "FAILED_SHORT_CIRCUIT" in data["message"]

        # 3. Try to run simulation with this faulty circuit in assembly_definition.yaml
        # First write assembly_definition.yaml
        assembly_def = {
            "version": "1.0",
            "constraints": {
                "benchmark_max_unit_cost_usd": 100,
                "benchmark_max_weight_g": 1000,
                "planner_target_max_unit_cost_usd": 80,
                "planner_target_max_weight_g": 800,
            },
            "electronics": faulty_section,
            "totals": {
                "estimated_unit_cost_usd": 50,
                "estimated_weight_g": 500,
                "estimate_confidence": "high",
            },
        }

        import yaml

        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "assembly_definition.yaml",
                "content": yaml.dump(assembly_def),
                "overwrite": True,
            },
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
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "script.py", "content": script, "overwrite": True},
            headers={"X-Session-ID": session_id},
        )

        # 4. Trigger simulation on worker_heavy
        sim_resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/simulate",
            json={"script_path": "script.py"},
            headers={"X-Session-ID": session_id},
            timeout=300.0,
        )
        assert sim_resp.status_code == 200
        sim_data = sim_resp.json()
        assert sim_data["success"] is False
        assert "FAILED_SHORT_CIRCUIT" in sim_data["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_121_short_circuit_detection():
    """INT-121: Verify short circuit detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = get_session_id("121")
        short_circuit = {
            "power_supply": {"voltage_dc": 12.0, "max_current_a": 1.0},
            "components": [],
            "wiring": [
                {
                    "wire_id": "short_wire_121",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "supply", "terminal": "0"},
                    "gauge_awg": 10,
                    "length_mm": 10000.0,
                }
            ],
        }
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
            json={"section": short_circuit},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        assert resp.json()["success"] is False
        assert "FAILED_SHORT_CIRCUIT" in resp.json()["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_122_overcurrent_supply_detection():
    """INT-122: Verify overcurrent supply detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = get_session_id("122")
        # Motor with 5A stall on a 1A PSU. Resistance = 12/5 = 2.4 Ohm.
        overcurrent = {
            "power_supply": {"voltage_dc": 12.0, "max_current_a": 1.0},
            "components": [
                {"component_id": "m1", "type": "motor", "stall_current_a": 5.0}
            ],
            "wiring": [
                {
                    "wire_id": "w1",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "m1", "terminal": "+"},
                    "gauge_awg": 18,
                    "length_mm": 500.0,
                },
                {
                    "wire_id": "w2",
                    "from": {"component": "m1", "terminal": "-"},
                    "to": {"component": "supply", "terminal": "0"},
                    "gauge_awg": 18,
                    "length_mm": 500.0,
                },
            ],
        }
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
            json={"section": overcurrent},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        assert resp.json()["success"] is False
        assert "FAILED_OVERCURRENT_SUPPLY" in resp.json()["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_123_overcurrent_wire_detection():
    """INT-123: Verify overcurrent wire detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = get_session_id("123")
        # High current (10A) on a very thin wire (AWG 30)
        # AWG 30 rated for ~0.5A
        wire_overcurrent = {
            "power_supply": {"voltage_dc": 12.0, "max_current_a": 20.0},
            "components": [
                {"component_id": "m1", "type": "motor", "stall_current_a": 10.0}
            ],
            "wiring": [
                {
                    "wire_id": "thin_wire",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "m1", "terminal": "+"},
                    "gauge_awg": 30,
                    "length_mm": 10.0,
                },
                {
                    "wire_id": "w2",
                    "from": {"component": "m1", "terminal": "-"},
                    "to": {"component": "supply", "terminal": "0"},
                    "gauge_awg": 18,
                    "length_mm": 10.0,
                },
            ],
        }
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
            json={"section": wire_overcurrent},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        assert resp.json()["success"] is False
        assert "FAILED_OVERCURRENT_WIRE" in resp.json()["message"]


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_124_open_circuit_detection():
    """INT-124: Verify open circuit / floating node detection via API."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = get_session_id("124")
        # Motor with only one terminal connected
        open_circuit = {
            "power_supply": {"voltage_dc": 12.0, "max_current_a": 10.0},
            "components": [
                {"component_id": "m1", "type": "motor", "stall_current_a": 2.0}
            ],
            "wiring": [
                {
                    "wire_id": "w1",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "m1", "terminal": "+"},
                    "gauge_awg": 22,
                    "length_mm": 100.0,
                }
                # Missing wire from m1:- to supply:0
            ],
        }
        resp = await client.post(
            f"{WORKER_HEAVY_URL}/benchmark/validate_circuit",
            json={"section": open_circuit},
            headers={"X-Session-ID": session_id},
        )
        assert resp.status_code == 200
        assert resp.json()["success"] is False
        assert "FAILED_OPEN_CIRCUIT" in resp.json()["message"]
