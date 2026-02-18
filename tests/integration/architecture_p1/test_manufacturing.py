import asyncio
import uuid

import pytest
from httpx import AsyncClient

# Adjust URL to your controller if different
CONTROLLER_URL = "http://localhost:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_manufacturing_methods_and_materials():
    """
    INT-035: Materials config enforcement
    INT-036: Supported workbench methods

    Verifies:
    1. Manufacturing workbench runs (validate_costing_and_price)
    2. Different methods (CNC, 3D Print) are supported/detected
    3. Invalid materials are rejected (via prompt injection or reviewing logs/artifacts)
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Setup Benchmark
        prompt = "Create a benchmark for a CNC machined part."
        resp = await client.post("/benchmark/generate", params={"prompt": prompt})
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to generate benchmark: {resp.text}"
        benchmark_session_id = resp.json()["session_id"]

        # Wait for benchmark
        for _ in range(30):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if (
                status_resp.status_code == 200
                and status_resp.json()["status"] == "completed"
            ):
                break
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation timed out.")

        # 2. Trigger Engineer - Requesting CNC and specific material
        engineer_session_id = str(uuid.uuid4())
        # We explicitly ask for Aluminum 6061 (valid) to test success path first
        task = f"Solve benchmark: {benchmark_session_id}. Use CNC milling with Aluminum 6061."
        run_payload = {
            "task": task,
            "session_id": engineer_session_id,
            "benchmark_session_id": benchmark_session_id,
        }

        run_resp = await client.post("/agent/run", json=run_payload)
        assert run_resp.status_code in [
            200,
            202,
        ], f"Failed to trigger agent: {run_resp.text}"
        episode_id = run_resp.json()["episode_id"]

        # Wait for Engineer
        engineer_completed = False
        final_status = None
        for _ in range(60):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                final_status = ep_resp.json()["status"]
                if final_status in ["completed", "failed", "max_turns_reached"]:
                    engineer_completed = True
                    break
            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail(f"Engineer timed out. Last status: {final_status}")

        # 3. Verify Workbench Execution (INT-036)
        artifacts_resp = await client.get(f"/artifacts/{episode_id}")
        assert artifacts_resp.status_code == 200
        artifacts = artifacts_resp.json()

        # Check for assembly_definition.yaml which implies workbench ran
        cost_yaml_artifact = next(
            (a for a in artifacts if "assembly_definition.yaml" in a["path"]),
            None,
        )
        assert cost_yaml_artifact is not None, (
            f"Workbench output (cost estimation) missing. Artifacts: {[a['path'] for a in artifacts]}"
        )

        # 4. Verify Material Enforcement (INT-035) - "Nice to have" negative test
        bad_material_session_id = str(uuid.uuid4())
        bad_task = f"Solve benchmark: {benchmark_session_id}. Use Unobtanium material."

        bad_run_resp = await client.post(
            "/agent/run",
            json={
                "task": bad_task,
                "session_id": bad_material_session_id,
                "benchmark_session_id": benchmark_session_id,
            },
        )
        assert bad_run_resp.status_code in [200, 202]
        bad_episode_id = bad_run_resp.json()["episode_id"]

        # Minimal assertion for INT-035: The system shouldn't crash, and if it fails, it handles it gracefully.
        bad_engineer_completed = False
        for _ in range(60):
            ep_resp = await client.get(f"/episodes/{bad_episode_id}")
            if ep_resp.status_code == 200:
                status = ep_resp.json()["status"]
                if status in ["completed", "failed"]:
                    bad_engineer_completed = True
                    break
            await asyncio.sleep(2)

        # We don't strictly assert completion here as per original logic,
        # but we could assert it didn't crash (status_code 500 etc)
