import asyncio
import os
import uuid

import httpx
import pytest
from pydantic import TypeAdapter

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import ListFilesRequest, ReadFileRequest, ReadFileResponse

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_045_skills_sync_lifecycle():
    """INT-045: Verify skills sync lifecycle."""
    async with httpx.AsyncClient(timeout=30.0) as client:
        # 1. Trigger Agent Run
        task = "Write a script that uses a skill"
        request = AgentRunRequest(
            task=task,
            session_id=f"INT-045-{uuid.uuid4().hex[:8]}",
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=request.model_dump(),
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        # 2. Wait for it to start/complete
        max_retries = 30
        for _ in range(max_retries):
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                episode_data = EpisodeResponse.model_validate(status_resp.json())
                if episode_data.status in [
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                ]:
                    break
            await asyncio.sleep(1)

        # 3. Verify skills are present on the worker
        # Agent initialization copies skills to /skills directory on worker
        fs_ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="/skills").model_dump(),
            headers={"X-Session-ID": str(request.session_id)},
        )
        assert fs_ls_resp.status_code == 200
        files = TypeAdapter(list[FileInfo]).validate_python(fs_ls_resp.json())
        assert len(files) > 0

        # Check for a specific known skill
        assert any("build123d_cad_drafting_skill" in f.name for f in files)

        # 4. Verify skill content is readable
        skill_path = "/skills/build123d_cad_drafting_skill/SKILL.md"
        fs_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path=skill_path).model_dump(),
            headers={"X-Session-ID": str(request.session_id)},
        )
        assert fs_resp.status_code == 200
        content_data = ReadFileResponse.model_validate(fs_resp.json())
        assert "# Build123d CAD Drafting Skill" in content_data.content
