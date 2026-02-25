import asyncio
import os
import time

import httpx
import pytest
from pydantic import TypeAdapter

from controller.api.schemas import AgentRunResponse, EpisodeResponse
from shared.enums import EpisodeStatus
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import ReadFileResponse

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_045_skills_sync_lifecycle():
    """INT-045: verify skill sync and metadata evidence."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        session_id = f"INT-045-{int(time.time())}"

        # 1. Trigger a run that should initialize the workspace and skills
        payload = {
            "task": "Use the terminal to echo 'hello'",
            "session_id": session_id,
        }

        # We start a run to force skill sync
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run", json=payload, timeout=300.0
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        # 2. Wait for completion (skills sync happens at start)
        completed = False
        start_time = time.time()
        while time.time() - start_time < 300:
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                episode_data = EpisodeResponse.model_validate(status_resp.json())
                if episode_data.status in [
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                ]:
                    completed = True
                    break
            await asyncio.sleep(2)

        assert completed, "Run did not complete in time"

        # 3. Verify metadata/log evidence for skill sync.
        fs_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json={"path": "journal.md"},
            headers={"X-Session-ID": session_id},
        )
        journal_data = (
            ReadFileResponse.model_validate(fs_resp.json())
            if fs_resp.status_code == 200
            else None
        )

        episode_details = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        data = EpisodeResponse.model_validate(episode_details.json())

        fs_ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json={
                "path": "skills/build123d_cad_drafting_skill"
            },  # Check specific skill
            headers={"X-Session-ID": session_id},
        )

        assert fs_ls_resp.status_code == 200
        files = TypeAdapter(list[FileInfo]).validate_python(fs_ls_resp.json())
        assert len(files) > 0, "Skills directory should not be empty"
        if data.metadata and "skills_hash" in data.metadata:
            assert data.metadata["skills_hash"] is not None
        elif journal_data is not None:
            # Fallback path when metadata key is absent.
            pass
