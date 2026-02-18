import asyncio
import time
from unittest.mock import MagicMock, patch
from fastapi import FastAPI
import httpx

# We want to test the actual code in worker/api/routes.py
from worker.api.routes import router
app = FastAPI()
app.include_router(router)

async def monitor_lag():
    print("Monitor: started")
    while True:
        start = time.perf_counter()
        await asyncio.sleep(0.1)
        end = time.perf_counter()
        lag = end - start - 0.1
        if lag > 0.05:
            print(f"Monitor: Event loop lag detected: {lag:.4f}s")

async def run_test():
    # We need to mock the filesystem router dependency
    from worker.api.routes import get_router

    mock_fs_router = MagicMock()
    mock_fs_router.local_backend.root = MagicMock()
    mock_fs_router.local_backend._resolve.return_value = "dummy.py"

    app.dependency_overrides[get_router] = lambda: mock_fs_router

    # Patch the heavy calls
    # load_component_from_script is called synchronously in the current code
    # preview_design is called via to_thread in the current code

    def slow_load(**kwargs):
        print("Executing load_component_from_script (synchronously)...")
        time.sleep(1)
        return MagicMock()

    def slow_render(*args, **kwargs):
        print("Executing preview_design (should be in thread)...")
        time.sleep(2)
        return MagicMock()

    with patch("worker.api.routes.load_component_from_script", side_effect=slow_load), \
         patch("worker.api.routes.preview_design", side_effect=slow_render):

        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(transport=transport, base_url="http://test") as client:
            print("Sending request to /benchmark/preview...")

            # Start monitor in background
            monitor_task = asyncio.create_task(monitor_lag())

            # Send request
            start_time = time.perf_counter()
            response = await client.post(
                "/benchmark/preview",
                json={"script_path": "main.py", "pitch": 0, "yaw": 0},
                headers={"X-Session-ID": "test-session"}
            )
            end_time = time.perf_counter()

            print(f"Request finished in {end_time - start_time:.2f}s")
            print(f"Response status: {response.status_code}")

            # Give monitor a chance to run one last time
            await asyncio.sleep(0.2)
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass

if __name__ == "__main__":
    asyncio.run(run_test())
