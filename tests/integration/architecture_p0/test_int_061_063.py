import os
import time
import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")

@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_061_asset_serving_security():
    """INT-061: Asset serving security + session isolation contract."""
    async with httpx.AsyncClient() as client:
        session_a = f"test-61-a-{int(time.time())}"
        session_b = f"test-61-b-{int(time.time())}"

        # 1. Setup session A with a valid python file and an asset
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "part.py", "content": "import build123d\ndef build(): return None"},
            headers={"X-Session-ID": session_a},
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "part.stl", "content": "dummy-stl-content"},
            headers={"X-Session-ID": session_a},
        )

        # 2. Setup session B with a broken python file and an asset
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "broken.py", "content": "invalid syntax !!!"},
            headers={"X-Session-ID": session_b},
        )
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "broken.stl", "content": "dummy-stl-content"},
            headers={"X-Session-ID": session_b},
        )

        # 3. Assert successful serving and MIME type for session A
        resp = await client.get(
            f"{WORKER_LIGHT_URL}/assets/part.stl",
            headers={"X-Session-ID": session_a}
        )
        assert resp.status_code == 200
        assert resp.content == b"dummy-stl-content"
        assert resp.headers["content-type"] == "model/stl"

        # 4. Assert 422 for session B due to syntax error in broken.py
        resp = await client.get(
            f"{WORKER_LIGHT_URL}/assets/broken.stl",
            headers={"X-Session-ID": session_b}
        )
        assert resp.status_code == 422
        assert "has syntax errors" in resp.text

        # 5. Assert session isolation: session B cannot access session A's assets
        resp = await client.get(
            f"{WORKER_LIGHT_URL}/assets/part.stl",
            headers={"X-Session-ID": session_b}
        )
        assert resp.status_code == 404

        # 6. Verify other MIME types
        await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "test.png", "content": "png-data"},
            headers={"X-Session-ID": session_a},
        )
        resp = await client.get(
            f"{WORKER_LIGHT_URL}/assets/test.png",
            headers={"X-Session-ID": session_a}
        )
        assert resp.status_code == 200
        assert resp.headers["content-type"] == "image/png"

@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_062_worker_openapi_contract():
    """INT-062: Split-worker OpenAPI artifact contract."""
    async with httpx.AsyncClient() as client:
        # Check Worker Light OpenAPI
        resp = await client.get(f"{WORKER_LIGHT_URL}/openapi.json")
        assert resp.status_code == 200
        light_schema = resp.json()
        paths = light_schema.get("paths", {})
        assert "/fs/ls" in paths
        assert "/assets/{path}" in paths

        # Check Worker Heavy OpenAPI
        resp = await client.get(f"{WORKER_HEAVY_URL}/openapi.json")
        assert resp.status_code == 200
        heavy_schema = resp.json()
        paths = heavy_schema.get("paths", {})
        assert "/benchmark/simulate" in paths
        assert "/benchmark/validate" in paths

@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_063_mounted_path_read_only():
    """INT-063: Mounted path compatibility/read-only contract."""
    async with httpx.AsyncClient() as client:
        session_id = f"test-63-{int(time.time())}"

        # 1. Verify workspace root is writable
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "writable.txt", "content": "i am writable"},
            headers={"X-Session-ID": session_id}
        )
        assert resp.status_code == 200

        # 2. Verify read-only mounts
        for ro_path in ["/utils", "/skills", "/reviews", "/config"]:
            # Try write
            resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/write",
                json={"path": f"{ro_path}/test.txt", "content": "fail"},
                headers={"X-Session-ID": session_id}
            )
            assert resp.status_code == 403, f"Expected 403 for {ro_path} write"

            # Try delete (if exists, or even if it doesn't, it should be blocked by prefix)
            resp = await client.post(
                f"{WORKER_LIGHT_URL}/fs/delete",
                json={"path": f"{ro_path}/nonexistent"},
                headers={"X-Session-ID": session_id}
            )
            assert resp.status_code == 403, f"Expected 403 for {ro_path} delete"
