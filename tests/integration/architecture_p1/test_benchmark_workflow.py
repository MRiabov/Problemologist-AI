import asyncio

import pytest
from httpx import AsyncClient

# Adjust URL to your controller if different
CONTROLLER_URL = "http://localhost:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_planner_cad_reviewer_path():
    """
    INT-031: Benchmark planner -> CAD -> reviewer path

    Verifies:
    1. Benchmark generation trigger
    2. Successful completion of the workflow
    3. Existence of required artifacts (plan.md, objectives.yaml, Reviews)
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=120.0) as client:
        # 1. Trigger Benchmark Generation
        prompt = "Create a simple path planning benchmark with a wall and a goal."
        resp = await client.post("/benchmark/generate", json={"prompt": prompt})
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to trigger benchmark: {resp.text}"
        data = resp.json()
        session_id = str(data["session_id"])
        # Override with INT-031 prefix for mock routing if needed,
        # but the controller generated a UUID.
        # Actually, the mock LM handles UUIDs by routing to 'benchmark' scenario.
        # If I want to use 'INT-031' specifically, I should probably pass it if the API allowed,
        # but /benchmark/generate generates its own UUID.
        # Wait, the MockDSPyLM routes UUIDs to 'benchmark'.
        # 'benchmark' in mock_responses.yaml is already defined.
        # Let's check 'benchmark' scenario in mock_responses.yaml.
        max_retries = 60
        benchmark_completed = False
        last_status = None

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = status_resp.json()
                last_status = sess_data["status"]

                if last_status == "completed":
                    benchmark_completed = True
                    break
                if last_status == "rejected" or last_status == "failed":
                    pytest.fail(
                        f"Benchmark generation failed with status: {last_status}"
                    )

            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail(f"Benchmark generation timed out. Last status: {last_status}")

        # 3. Verify Artifacts
        artifacts_resp = await client.get(f"/artifacts/{session_id}")
        assert artifacts_resp.status_code == 200, (
            f"Failed to fetch artifacts: {artifacts_resp.text}"
        )
        artifacts_list = artifacts_resp.json()

        # Check for key files
        artifact_paths = [a["path"] for a in artifacts_list]

        assert any(p.endswith("plan.md") for p in artifact_paths), (
            f"plan.md missing. Artifacts: {artifact_paths}"
        )
        assert any(p.endswith("objectives.yaml") for p in artifact_paths), (
            f"objectives.yaml missing. Artifacts: {artifact_paths}"
        )
        assert any("reviews/" in p for p in artifact_paths), (
            f"Reviews missing. Artifacts: {artifact_paths}"
        )
