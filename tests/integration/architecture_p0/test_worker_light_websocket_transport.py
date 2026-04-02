import os
import uuid
from pathlib import Path

import pytest

from controller.clients.worker import WorkerClient

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")


def _asset_path(asset_path: str) -> Path:
    return Path(str(asset_path).lstrip("/"))


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_worker_light_websocket_transport_round_trip():
    session_id = f"INT-WS-{uuid.uuid4().hex[:8]}"
    client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=WORKER_HEAVY_URL,
        light_transport="ws",
    )

    try:
        assert await client.write_file("script.py", 'print("ws transport")\n')

        listing = await client.list_files("/")
        assert any(_asset_path(entry.path) == Path("script.py") for entry in listing)

        read_back = await client.read_file("script.py")
        assert "ws transport" in read_back

        bundle = await client.bundle_session()
        assert len(bundle) > 0

        execution = await client.execute_command(
            "python script.py", timeout=30, episode_id=session_id
        )
        assert execution.exit_code == 0
        assert "ws transport" in execution.stdout

        assert await client.git_init()
        status = await client.git_status()
        assert status.branch
    finally:
        await client.aclose()
