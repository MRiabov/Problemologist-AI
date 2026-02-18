import asyncio
import httpx
import json
import sys
import pytest

WORKER_URL = "http://localhost:18001"

SCRIPT_1 = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.enums import ManufacturingMethod

def build(params=None):
    part = Box(10, 10, 10)
    part.label = "target_box"
    part.metadata = PartMetadata(material_id="aluminum_6061", manufacturing_method=ManufacturingMethod.THREE_DP)
    return part.move(Location((0, 0, 10)))
"""

SCRIPT_2 = """
from build123d import *
from shared.models.schemas import PartMetadata
from shared.enums import ManufacturingMethod

def build(params=None):
    part = Sphere(10)
    part.label = "target_sphere"
    part.metadata = PartMetadata(material_id="aluminum_6061", manufacturing_method=ManufacturingMethod.THREE_DP)
    return part.move(Location((0, 0, 15)))
"""


async def run_simulation(script: str, name: str):
    async with httpx.AsyncClient() as client:
        try:
            response = await client.post(
                f"{WORKER_URL}/benchmark/simulate",
                headers={"x-session-id": f"test-multi-{name}"},
                json={
                    "session_id": f"test-multi-{name}",
                    "script_content": script,
                    "backend": "genesis",
                    "smoke_test_mode": True,
                },
                timeout=60.0,
            )
            return {
                "name": name,
                "status": response.status_code,
                "data": response.json() if response.status_code == 200 else None,
                "error": response.text if response.status_code != 200 else None,
            }
        except Exception as e:
            import traceback

            return {
                "name": name,
                "success": False,
                "error": f"{type(e).__name__}: {str(e)}\n{traceback.format_exc()}",
            }


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_multitenancy_repro():
    # Pre-warm: the first simulation on a fresh worker always takes long due to kernel compilation.
    # We run a tiny dummy simulation to warm up the pool
    # and we include both Box and Sphere to trigger all needed kernels.
    print("Pre-warming Genesis cache...")
    warmup_script = """
from build123d import Box, Sphere, Compound
from shared.models.schemas import PartMetadata
from shared.enums import ManufacturingMethod
def build():
    b = Box(1, 1, 1)
    b.metadata = PartMetadata(material_id="aluminum_6061", manufacturing_method=ManufacturingMethod.THREE_DP)
    s = Sphere(1)
    s.metadata = PartMetadata(material_id="aluminum_6061", manufacturing_method=ManufacturingMethod.THREE_DP)
    return b + s
"""
    async with httpx.AsyncClient() as client:
        print("Warmup round...")
        await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            headers={"x-session-id": "warmup"},
            json={
                "session_id": "warmup",
                "script_content": warmup_script,
                "smoke_test_mode": True,
            },
            timeout=300.0,
        )
    print("Warmup complete. Running actual tests...")

    # Run sequentially to avoid resource contention on CPU-only hardware
    # while still verifying session isolation and backend reuse.
    res1 = await run_simulation(SCRIPT_1, "sim1_box")
    res2 = await run_simulation(SCRIPT_2, "sim2_sphere")

    results = [res1, res2]

    for res in results:
        # Basic assertions
        assert (
            res.get("status") == 200
        ), f"FAILURE: {res['name']} returned {res.get('status')}. Error: {res.get('error')}. Data: {json.dumps(res.get('data'), indent=2)}"

        data = res.get("data", {})
        success = data.get("success", False)
        message = data.get("message", "")

        # Check success or specific stability message
        assert (
            success or "Simulation stable" in message
        ), f"FAILURE: {res['name']} reported success=False. Msg: {message}. Full Data: {json.dumps(data, indent=2)}"


if __name__ == "__main__":
    # Run the test logic manually
    asyncio.run(test_multitenancy_repro())


if __name__ == "__main__":
    if "httpx" not in sys.modules:
        # Fallback if I can't run this easily, but user environment has it
        pass
    # Run the test logic manually
    asyncio.run(test_multitenancy_repro())
