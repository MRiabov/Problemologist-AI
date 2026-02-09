import asyncio
import uuid
from pathlib import Path
import httpx
import pytest

# Assuming worker is running at http://localhost:8001
WORKER_URL = "http://localhost:8001"


async def run_session(session_id: str, filename: str, content: str):
    """
    Simulates a session:
    1. Write a file checks CWD isolation.
    2. Execute python code that reads the file.
    """
    headers = {"X-Session-ID": session_id}

    async with httpx.AsyncClient(timeout=30.0) as client:
        # 1. Write file
        resp = await client.post(
            f"{WORKER_URL}/fs/write",
            json={"path": filename, "content": content},
            headers=headers,
        )
        assert resp.status_code == 200, f"Write failed: {resp.text}"

        # 2. Execute code to read it from CWD
        code = f"""
import os
files = os.listdir('.')
print(f"Files in CWD: {{files}}")
with open("{filename}", "r") as f:
    print(f.read())
"""
        resp = await client.post(
            f"{WORKER_URL}/runtime/execute",
            json={"code": code, "timeout": 5},
            headers=headers,
        )
        assert resp.status_code == 200, f"Execute failed: {resp.text}"
        data = resp.json()
        assert data["exit_code"] == 0
        assert content in data["stdout"]
        return data["stdout"]


@pytest.mark.integration
@pytest.mark.asyncio
async def test_worker_concurrency():
    session_a = f"session-{uuid.uuid4()}"
    session_b = f"session-{uuid.uuid4()}"

    file_a = "secret_a.txt"
    content_a = "Content for session A"

    file_b = "secret_b.txt"
    content_b = "Content for session B"

    # Run both concurrently
    task_a = run_session(session_a, file_a, content_a)
    task_b = run_session(session_b, file_b, content_b)

    results = await asyncio.gather(task_a, task_b)

    stdout_a, stdout_b = results

    # Verify Isolation
    # Session A should NOT see file_b
    assert file_a in stdout_a
    assert file_b not in stdout_a

    # Session B should NOT see file_a
    assert file_b in stdout_b
    assert file_a not in stdout_b

    print("Concurrency test passed!")


if __name__ == "__main__":
    asyncio.run(test_worker_concurrency())
