import asyncio
import time
import httpx
import pytest

# Adjust URL to your worker
WORKER_URL = "http://localhost:8001"

SCRIPT_CONTENT = """
from build123d import *
def build():
    return Compound(children=[Box(10,10,10)])
"""


async def run_simulation(session_id: str):
    async with httpx.AsyncClient(timeout=60.0) as client:
        start_time = time.time()
        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={
                "script_path": "test_box.py",
                "script_content": SCRIPT_CONTENT,
                "command": "simulate",
            },
            headers={"X-Session-ID": session_id},
        )
        end_time = time.time()
        return resp, start_time, end_time


@pytest.mark.asyncio
async def test_simulation_concurrency():
    # We want to verify that simulations are serialized.
    # We'll launch 3 simulations.
    # If they run in parallel, they might finish in roughly max(duration).
    # If serialized, they finish in sum(duration).

    # Note: This relies on simulate taking non-trivial time.
    # The current simulate implementation does 100 mujoco steps and 24 renders,
    # so it should be slow enough.

    session_ids = ["sess_1", "sess_2", "sess_3"]
    tasks = [run_simulation(sid) for sid in session_ids]

    print("Starting 3 concurrent simulations...")
    global_start = time.time()
    results = await asyncio.gather(*tasks)
    global_end = time.time()

    total_duration = global_end - global_start
    print(f"Total duration: {total_duration:.2f}s")

    durations = []
    for i, (resp, start, end) in enumerate(results):
        assert resp.status_code == 200, f"Sim {i} failed: {resp.text}"
        assert resp.json()["success"], (
            f"Sim {i} returned success=False: {resp.json().get('message')}"
        )
        duration = end - start
        durations.append(duration)
        print(
            f"Sim {i}: {duration:.2f}s (Start: {start - global_start:.2f}, End: {end - global_start:.2f})"
        )

    # Check for overlap
    # If serialized, the intervals [start, end] should not overlap significantly
    # (allowing for network latency/server processing overhead).
    # Simplified check: Sort by start time. End of one should be <= Start of next.

    sorted_intervals = sorted([(s, e) for _, s, e in results], key=lambda x: x[0])

    overlaps = 0
    for i in range(len(sorted_intervals) - 1):
        prev_end = sorted_intervals[i][1]
        next_start = sorted_intervals[i + 1][0]
        # Allow tiny overlap due to clock/logging noise, but massive overlap means parallel
        if next_start < prev_end - 0.5:  # 0.5s tolerance
            overlaps += 1
            print(
                f"Overlap detected: Sim {i} ended at {prev_end}, Sim {i + 1} started at {next_start}"
            )

    if overlaps == 0:
        print(
            "PASS: No significant file execution overlaps detected. Simulations appeared serialized."
        )
    else:
        print(
            f"FAIL: {overlaps} overlaps detected. Simulations appeared properly parallel (which is NOT what we want)."
        )
        # Fail the test if we want to enforce serialization
        pytest.fail("Simulations ran in parallel, expected serialization.")


if __name__ == "__main__":
    asyncio.run(test_simulation_concurrency())
