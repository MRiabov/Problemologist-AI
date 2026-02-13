import asyncio
import os
import time
import httpx
import pytest

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_043_batch_execution_path():
    """INT-043: Verify batch job submission and execution isolation."""
    async with httpx.AsyncClient() as client:
        # Submit 3 concurrent jobs
        tasks = []
        session_ids = []
        for i in range(2):
            session_id = f"batch-test-{int(time.time())}-{i}"
            session_ids.append(session_id)
            payload = {
                "task": f"Batch execution test task {i}",
                "session_id": session_id,
            }
            tasks.append(
                client.post(f"{CONTROLLER_URL}/agent/run", json=payload, timeout=10.0)
            )

        # 1. Verify all accepted
        responses = await asyncio.gather(*tasks)
        episode_ids = []
        for resp in responses:
            assert resp.status_code == 202
            episode_ids.append(resp.json()["episode_id"])

        assert len(set(episode_ids)) == 2, "All episode IDs must be unique"

        # 2. Verify eventual completion of all
        # We poll all episodes until they are done or we timeout
        start_time = time.time()
        timeout = 60  # seconds

        while time.time() - start_time < timeout:
            all_done = True
            for episode_id in episode_ids:
                status_resp = await client.get(
                    f"{CONTROLLER_URL}/episodes/{episode_id}", timeout=5.0
                )
                if status_resp.status_code != 200:
                    all_done = False
                    break

                status = status_resp.json()["status"]
                if status == "failed":
                    # Fetch more details if possible
                    detail_resp = await client.get(
                        f"{CONTROLLER_URL}/episodes/{episode_id}"
                    )
                    details = (
                        detail_resp.json()
                        if detail_resp.status_code == 200
                        else "No details"
                    )
                    pytest.fail(
                        f"Episode {episode_id} failed unexpectedly. Details: {details}"
                    )

                if (
                    status not in ["completed", "stopped"]
                ):  # stopped is also a terminal state if we cancelled it, but here we expect completion
                    all_done = False

            if all_done:
                break

            await asyncio.sleep(2)
        else:
            pytest.fail(
                f"Batch execution timed out after {timeout}s. Not all episodes completed."
            )

        # 3. Verify Per-Episode Isolation (Basic check: different session IDs)
        # We already ensured they have different session IDs and episode IDs.
        # A deeper check would be verifying filesystem isolation, but that is covered by INT-003.
        # Here we just ensure the controller tracked them separately.

        # Double check status one last time
        for episode_id in episode_ids:
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            assert status_resp.json()["status"] == "completed"
