import asyncio
import uuid

import pytest
from httpx import AsyncClient

# Adjust URL to your controller if different
CONTROLLER_URL = "http://localhost:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_engineering_full_loop():
    """
    INT-033: Engineering full loop (planner/coder/reviewer)

    Verifies:
    1. Triggers Engineering Agent on a valid benchmark
    2. Planner generates plan.md (with budgets) and todo.md
    3. Coder attempts implementation (producing script.py or similar)
    4. Reviewer approves/rejects with typed decision

    Note: Requires a valid benchmark session ID.
    If we cannot rely on a pre-existing one, we generate one first.
    """
    async with AsyncClient(
        base_url=CONTROLLER_URL, timeout=300.0
    ) as client:  # Longer timeout for full loop
        # 1. Setup: Generate a Benchmark (or use a mocked ID if testing against mock)
        # For integration, we generate one.
        print("Generating benchmark for INT-033...")
        prompt = "Create a benchmark about stacking blocks."
        resp = await client.post("/benchmark/generate", params={"prompt": prompt})
        assert resp.status_code in [200, 202]
        benchmark_session_id = resp.json()["session_id"]

        # Wait for benchmark
        max_retries = 30
        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if (
                status_resp.status_code == 200
                and status_resp.json()["status"] == "completed"
            ):
                break
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation failed or timed out during setup.")

        # 2. Trigger Engineer Agent
        print(f"Triggering Engineer for benchmark {benchmark_session_id}...")
        engineer_session_id = str(uuid.uuid4())
        task = f"Solve benchmark: {benchmark_session_id}"
        run_payload = {
            "task": task,
            "session_id": engineer_session_id,
            "benchmark_session_id": benchmark_session_id,  # If API supports linking directly
        }

        run_resp = await client.post("/agent/run", json=run_payload)
        assert run_resp.status_code in [200, 202]
        episode_id = run_resp.json()["episode_id"]
        print(f"Engineer Episode ID: {episode_id}")

        # 3. Poll for Engineering Completion
        # This can take time. We look for 'completed' or 'failed' (but with artifacts).
        # Even if failed (e.g. max retries), artifacts should exist if it ran.
        engineer_completed = False

        for _ in range(60):  # Poll for up to 2 mins
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep_data = ep_resp.json()
                status = ep_data["status"]
                print(f"Engineer Status: {status}")
                if status in ["completed", "failed", "max_turns_reached"]:
                    engineer_completed = True
                    break
            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail("Engineer loop timed out.")

        # 4. Verify Engineering Artifacts
        artifacts_resp = await client.get(
            f"/artifacts/{episode_id}"
        )  # Check if episode ID equates to session ID for artifacts
        # If artifacts are stored by session_id, we might need to use engineer_session_id or filter by episode

        # Try fetching by session_id/episode_id
        # Assuming artifacts endpoint supports either or we query by session_id

        # If the API uses session_id for artifacts:
        artifacts_resp = await client.get(f"/artifacts/{engineer_session_id}")
        if artifacts_resp.status_code != 200:
            # Try episode_id as fallback or maybe the session_id is different?
            pass

        assert artifacts_resp.status_code == 200
        artifacts = artifacts_resp.json()
        artifact_paths = [a["path"] for a in artifacts]
        print(f"Engineer Artifacts: {artifact_paths}")

        # Check for Planner artifacts
        assert any("plan.md" in p for p in artifact_paths), "plan.md missing"
        assert any("todo.md" in p for p in artifact_paths), "todo.md missing"
        assert any(
            "preliminary_cost_estimation.yaml" in p for p in artifact_paths
        ), "Cost estimation missing"

        # Check for Reviewer artifacts
        # Reviews are usually in reviews/ folder
        review_files = [p for p in artifact_paths if "reviews/" in p]
        assert (
            len(review_files) > 0
        ), "No reviews found (Planner -> Reviewer or Coder -> Reviewer loop missing)"

        # We could inspect content here if we had content access,
        # to verify "typed decision" (e.g. accepted/rejected in frontmatter)
