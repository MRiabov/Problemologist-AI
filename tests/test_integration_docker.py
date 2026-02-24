import asyncio
import os
import time

import httpx
import pytest

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_services_health():
    """Verify both controller and worker are healthy."""
    async with httpx.AsyncClient() as client:
        # Check Controller
        try:
            resp = await client.get(f"{CONTROLLER_URL}/health", timeout=10.0)
            assert resp.status_code == 200
            # main.py defines health as {"status": "healthy"} or ResponseStatus.HEALTHY
            assert resp.json()["status"].lower() in ["healthy", "ok"]
        except Exception as e:
            pytest.fail(f"Controller at {CONTROLLER_URL} is not reachable: {e}")

        # Check Worker
        try:
            resp = await client.get(f"{WORKER_LIGHT_URL}/health", timeout=10.0)
            assert resp.status_code == 200
            assert resp.json()["status"].lower() in ["healthy", "ok"]
        except Exception as e:
            pytest.fail(f"Worker at {WORKER_LIGHT_URL} is not reachable: {e}")


@pytest.mark.integration
@pytest.mark.asyncio
async def test_controller_to_worker_agent_run():
    """Verify controller can trigger an agent run which talks to the worker."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-agent-{int(time.time())}"
        payload = {
            "task": "Write a file named 'hello_integration.txt' with content 'integration test'",
            "session_id": session_id,
        }

        # Trigger agent run
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run", json=payload, timeout=10.0
        )
        assert resp.status_code in [200, 202]
        data = resp.json()
        assert data["status"] == "accepted"
        episode_id = data["episode_id"]

        # Wait for completion (poll episode status)
        # In a real integration test, we wait for the background task to finish
        max_retries = 30
        completed = False
        for _ in range(max_retries):
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                if status_resp.json()["status"] == "completed":
                    completed = True
                    break
                if status_resp.json()["status"] == "failed":
                    pytest.fail("Agent run failed in background")
            await asyncio.sleep(2)

        if not completed:
            pytest.fail("Agent run timed out")

        # Verify the file was actually written to the worker's filesystem
        fs_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "hello_integration.txt"},
            headers={"X-Session-ID": session_id},
        )
        assert fs_resp.status_code == 200
        assert fs_resp.json()["content"] == "integration test"
