import asyncio
import time
import httpx

WORKER_URL = "http://localhost:8001"

SCRIPT_CONTENT = """
from build123d import *
def build():
    return Compound(children=[Box(10,10,10)])
"""


async def run_single_simulation():
    async with httpx.AsyncClient(timeout=60.0) as client:
        print("Sending request...")
        resp = await client.post(
            f"{WORKER_URL}/benchmark/simulate",
            json={
                "script_path": "test_box_single.py",
                "script_content": SCRIPT_CONTENT,
                "command": "simulate",
            },
            headers={"X-Session-ID": "single_session"},
        )
        print(f"Response: {resp.status_code}")
        print(resp.text)


if __name__ == "__main__":
    asyncio.run(run_single_simulation())
