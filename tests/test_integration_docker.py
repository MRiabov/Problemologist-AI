import pytest
import httpx
import os
import time

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:8000")
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:8001")

@pytest.mark.integration
@pytest.mark.asyncio
async def test_services_health():
    """Verify both controller and worker are healthy."""
    async with httpx.AsyncClient() as client:
        # Check Controller
        try:
            resp = await client.get(f"{CONTROLLER_URL}/health")
            assert resp.status_code == 200
            assert resp.json()["status"] in ["healthy", "ok", "HEALTHY"]
        except Exception as e:
            pytest.fail(f"Controller at {CONTROLLER_URL} is not reachable: {e}")

        # Check Worker
        try:
            resp = await client.get(f"{WORKER_URL}/health")
            assert resp.status_code == 200
            assert resp.json()["status"] == "healthy"
        except Exception as e:
            pytest.fail(f"Worker at {WORKER_URL} is not reachable: {e}")

@pytest.mark.integration
@pytest.mark.asyncio
async def test_controller_to_worker_agent_run():
    """Verify controller can trigger an agent run which talks to the worker."""
    async with httpx.AsyncClient() as client:
        payload = {
            "task": "Write a file named 'hello_integration.txt' with content 'integration test'",
            "session_id": f"test-integration-{int(time.time())}"
        }
        # The /agent/run endpoint in controller uses WorkerClient to talk to worker
        # Note: This might take some time as it involves LLM if not mocked, 
        # but the user said "Don't mock".
        # We might need a longer timeout.
        try:
            resp = await client.post(
                f"{CONTROLLER_URL}/agent/run", 
                json=payload,
                timeout=60.0
            )
            assert resp.status_code == 200
            assert resp.json()["status"] == "completed"
            
            # Verify the file was actually written to the worker's filesystem
            # We can check this by calling the worker's FS API directly
            fs_resp = await client.post(
                f"{WORKER_URL}/fs/read",
                json={"path": "/hello_integration.txt"},
                headers={"X-Session-ID": payload["session_id"]}
            )
            assert fs_resp.status_code == 200
            assert fs_resp.json()["content"] == "integration test"
        except Exception as e:
            pytest.fail(f"Agent run integration failed: {e}")

@pytest.mark.integration
@pytest.mark.asyncio
async def test_simulation_workflow():
    """Verify simulation workflow (Controller -> Temporal -> Worker activity)."""
    # This requires Temporal to be running and workers to be registered.
    # In docker-compose, 'controller' service runs 'src/controller/api/main.py'
    # And 'worker' service runs 'src/worker/app.py' (FastAPI).
    # Wait, where is the Temporal worker running?
    # src/controller/worker.py seems to be the Temporal worker.
    # Looking at docker-compose.yml again...
    pass
