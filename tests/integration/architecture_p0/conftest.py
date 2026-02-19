import base64
import os
import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


@pytest.fixture
def get_bundle():
    """Fixture that returns a function to fetch gzipped workspace from light worker."""
    async def _get_bundle(client: httpx.AsyncClient, session_id: str) -> str:
        resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/bundle",
            headers={"X-Session-ID": session_id},
            timeout=60.0,
        )
        assert resp.status_code == 200, f"Failed to get bundle: {resp.text}"
        return base64.b64encode(resp.content).decode("utf-8")

    return _get_bundle
