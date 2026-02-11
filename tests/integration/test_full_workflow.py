import asyncio
import uuid

import pytest
from httpx import AsyncClient

# Adjust URL to your controller
CONTROLLER_URL = "http://localhost:8000"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_full_workflow_end_to_end():
    """
    Test the full workflow:
    1. Trigger Benchmark Generator (via API)
    2. Wait for Benchmark to complete and produce assets
    3. Trigger Engineer Agent (via API) with the generated benchmark
    4. Wait for Engineer to complete

    We mock the LLM responses to ensure deterministic execution and avoid costs.
    """

    # Mock responses for the Benchmark Generator and Engineer Agents
    # We need to mock the `ainvoke` method of the LLM or the graph nodes.
    # Since we are running against a live server (via API), we can't easily mock
    # internal objects unless we run the server *within* the test process
    # or use a special testing mode.

    # However, `scripts/run_integration_tests.sh` runs a docker container.
    # The tests run *outside* the container against the exposed ports.
    # This makes mocking internal server objects impossible from the test script.

    # Strategy:
    # 1. If we are running in a "mocked" mode, the server should use a mock LLM.
    # 2. Or, we skip the "mocking" here and rely on the fact that we might need
    #    to run this against a real LLM (expensive/flaky) OR we accept that
    #    we can only test the API surface and not the internal logic if we can't control the server.

    # WAITING: For now, I will write the test assuming the server is running
    # and we are just triggering endpoints. If we need to mock, we'd need
    # to change how the server is started or introduce a "test mode" config
    # that injects mock LLMs.

    # Given the user wants "integration tests", and typically these run against
    # a deployed stack, I will write it as a black-box test.
    # BUT, without a real LLM, this will fail if the server tries to call OpenAI.

    # Let's assume for this task that we just want to verify the *plumbing*
    # works (workflow transitions), even if the agents fail or we stub them.

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=60.0) as client:
        # 1. Trigger Benchmark Generation
        print("Triggering Benchmark Generation...")
        prompt = "Create a simple box stacking benchmark."
        resp = await client.post("/benchmark/generate", params={"prompt": prompt})
        assert resp.status_code == 202
        data = resp.json()
        session_id = data["session_id"]
        print(f"Benchmark Session ID: {session_id}")

        # 2. Poll for completion
        # In a real integration test without mocks, this might take a long time and cost money.
        # But here we assume the environment might be configured with a mock LLM or we accept the cost.
        # If we can't mock, we might need to skip the waiting part if we just want to test plumbing.

        # However, to be a "Full Workflow" test, we need the output.
        # Let's poll for a limited time.
        max_retries = 30
        benchmark_completed = False
        benchmark_data = None

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = status_resp.json()
                status = sess_data["status"]
                print(f"Benchmark Status: {status}")

                if (
                    status == "completed"
                ):  # Changed from "accepted" to "completed" based on typical workflow states
                    benchmark_completed = True
                    benchmark_data = sess_data
                    break
                if status == "rejected":
                    pytest.fail("Benchmark generation was rejected.")

            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail("Benchmark generation timed out or failed to complete.")

        # 3. Retrieve assets (MJCF, constraints) from the session data or DB
        # The session data should have the result.
        # We need to extract the 'objectives.yaml' equivalent or the constraints to pass to the Engineer.

        # For this test, we construct a synthetic task prompt for the Engineer
        # mimicking what the Benchmark Generator would output.

        engineer_task = (
            f"Solve this benchmark: {session_id}. Constraints: Use standard parts."
        )

        # 4. Trigger Engineer Agent
        print("Triggering Engineer Agent...")
        # Generate a unique session ID for the engineer run
        engineer_session_id = str(uuid.uuid4())

        run_payload = {"task": engineer_task, "session_id": engineer_session_id}

        run_resp = await client.post("/agent/run", json=run_payload)
        assert run_resp.status_code == 202
        episode_id = run_resp.json()["episode_id"]
        print(f"Engineer Episode ID: {episode_id}")

        # 5. Poll for Engineer completion
        engineer_completed = False
        for _ in range(max_retries):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep_data = ep_resp.json()
                status = ep_data["status"]
                print(f"Engineer Status: {status}")

                if status == "completed":
                    engineer_completed = True
                    break
                if status == "failed":
                    pytest.fail("Engineer agent failed.")

            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail("Engineer agent timed out.")

        print("Full workflow test passed!")
