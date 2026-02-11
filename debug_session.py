import asyncio
import uuid
import httpx
import json


async def debug_session_state():
    url = "http://localhost:8001"
    session_id = f"test-debug-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    async with httpx.AsyncClient(base_url=url, timeout=10.0) as client:
        print(f"--- Debugging Session: {session_id} ---")

        # 1. Check initial state
        resp = await client.get("/fs/ls", params={"path": "/"}, headers=headers)
        print(f"Initial LS: {json.dumps(resp.json(), indent=2)}")

        # 2. Try to submit immediately
        print("Attempting submission without any files...")
        submit_payload = {
            "script_path": "any.py",
            "script_content": "def build(): return None",
        }
        resp = await client.post(
            "/benchmark/submit", json=submit_payload, headers=headers
        )
        print(f"Submit response: {resp.status_code}, {resp.json()}")


if __name__ == "__main__":
    asyncio.run(debug_session_state())
