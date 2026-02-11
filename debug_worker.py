import asyncio
import uuid
import httpx


async def debug_worker():
    url = "http://localhost:8001"
    session_id = f"debug-{uuid.uuid4().hex[:8]}"
    headers = {"X-Session-ID": session_id}

    async with httpx.AsyncClient(base_url=url, timeout=10.0) as client:
        # 1. Write a file
        print(f"Writing test.txt to session {session_id}...")
        resp = await client.post(
            "/fs/write", json={"path": "test.txt", "content": "hello"}, headers=headers
        )
        print(f"Write response: {resp.status_code}, {resp.json()}")

        # 2. List files
        print("Listing files...")
        resp = await client.get("/fs/ls", params={"path": "/"}, headers=headers)
        print(f"LS response: {resp.status_code}, {resp.json()}")

        # 3. Read back
        print("Reading back...")
        resp = await client.get(
            "/fs/read", params={"path": "test.txt"}, headers=headers
        )
        print(f"Read response: {resp.status_code}, {resp.text[:100]}")


if __name__ == "__main__":
    asyncio.run(debug_worker())
