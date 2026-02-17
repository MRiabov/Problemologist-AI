import asyncio
import time

import httpx
import pytest

# Adjust URL to your worker
WORKER_URL = "http://localhost:18001"

# A simple valid script that does something
SCRIPT_CONTENT = """
from build123d import *
from shared.models.schemas import PartMetadata
def build():
    b = Box(10, 10, 10)
    b.label = "test_box"
    b.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return b
"""


async def run_simulation(session_id: str):
    async with httpx.AsyncClient(timeout=120.0) as client:
        start_time = time.time()
        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={
                "script_path": "test_box.py",
                "script_content": SCRIPT_CONTENT,
            },
            headers={"X-Session-ID": session_id},
        )
        end_time = time.time()
        return resp, start_time, end_time


@pytest.mark.integration
@pytest.mark.asyncio
async def test_simulation_concurrency_serialization():
    # We want to verify that simulations are SERIALIZED.
    # Even if we launch 3 concurrent requests, they should happen one after another
    # because of the HEAVY_OPERATION_LOCK.

    session_ids = [f"sess_serialize_{i}_{int(time.time())}" for i in range(3)]

    # Pre-write the scripts
    async with httpx.AsyncClient() as client:
        for sid in session_ids:
            await client.post(
                f"{WORKER_URL}/fs/write",
                json={"path": "test_box.py", "content": SCRIPT_CONTENT},
                headers={"X-Session-ID": sid},
            )

    print("\nStarting 3 concurrent simulation requests (expecting serialization)...")
    global_start = time.time()
    results = await asyncio.gather(*[run_simulation(sid) for sid in session_ids])
    global_end = time.time()

    total_duration = global_end - global_start
    print(f"Total real-time duration: {total_duration:.2f}s")

    durations = []
    intervals = []
    for i, (resp, start, end) in enumerate(results):
        assert resp.status_code == 200, f"Sim {i} failed: {resp.text}"
        data = resp.json()
        assert data["success"], f"Sim {i} returned success=False: {data.get('message')}"

        duration = end - start
        durations.append(duration)
        intervals.append((start, end))
        print(
            f"Sim {i}: {duration:.2f}s (Start: {start - global_start:.2f}, End: {end - global_start:.2f})"
        )

    sum_durations = sum(durations)
    print(f"Sum of individual durations: {sum_durations:.2f}s")

    # If serialized, total_duration should be >= sum_durations (roughly)
    # If parallel, total_duration would be ~ max(durations)
    assert (
        total_duration >= sum_durations * 0.9
    ), f"Simulations appeared parallel. Total: {total_duration:.2f}s, Sum: {sum_durations:.2f}s"

    # Check for significant overlaps
    sorted_intervals = sorted(intervals, key=lambda x: x[0])
    overlaps = 0
    for i in range(len(sorted_intervals) - 1):
        prev_end = sorted_intervals[i][1]
        next_start = sorted_intervals[i + 1][0]
        # Allowing 0.5s for network/FastAPI overhead
        if next_start < prev_end - 0.5:
            overlaps += 1
            print(
                f"Overlap detected: Sim {i} ended at {prev_end - global_start:.2f}s, "
                f"Sim {i + 1} started at {next_start - global_start:.2f}s"
            )

    assert overlaps == 0, f"Serialization failed: {overlaps} overlaps detected."
    print(
        "PASS: No significant overlaps detected. Parallelism avoided, serialization verified."
    )


if __name__ == "__main__":
    asyncio.run(test_simulation_concurrency_serialization())
