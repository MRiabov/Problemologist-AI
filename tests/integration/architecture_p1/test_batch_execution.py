import asyncio
import os
import time

import httpx
import pytest

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_043_batch_execution_path():
    """INT-043: Verify batch job submission and execution isolation."""
    async with httpx.AsyncClient(timeout=300.0) as client:
        # Submit 2 concurrent jobs
        tasks = []
        session_ids = []
        for i in range(2):
            session_id = f"INT-043-batch-{i}-{int(time.time())}"
            session_ids.append(session_id)
            request = AgentRunRequest(
                task=f"Batch execution test task {i}",
                session_id=session_id,
            )
            tasks.append(
                client.post(f"{CONTROLLER_URL}/agent/run", json=request.model_dump(), timeout=10.0)
            )

        # 1. Verify all accepted
        responses = await asyncio.gather(*tasks)
        episode_ids = []
        for resp in responses:
            assert resp.status_code == 202
            agent_run_resp = AgentRunResponse.model_validate(resp.json())
            episode_ids.append(agent_run_resp.episode_id)

        assert len(set(episode_ids)) == 2, "All episode IDs must be unique"

        # 2. Verify eventual completion of all
        # We poll all episodes until they are done or we timeout
        start_time = time.time()
        timeout = 300  # seconds

        while time.time() - start_time < timeout:
            all_done = True
            for episode_id in episode_ids:
                status_resp = await client.get(
                    f"{CONTROLLER_URL}/episodes/{episode_id}", timeout=5.0
                )  # should be instant because LLMs are mocked.
                if status_resp.status_code != 200:
                    all_done = False
                    break

                ep_data = EpisodeResponse.model_validate(status_resp.json())
                status = ep_data.status
                if status == EpisodeStatus.FAILED:
                    pytest.fail(
                        f"Episode {episode_id} failed unexpectedly. Details: {ep_data}"
                    )

                if (
                    status not in [EpisodeStatus.COMPLETED, EpisodeStatus.CANCELLED]
                ):
                    all_done = False

            if all_done:
                break

            await asyncio.sleep(2)
        else:
            pytest.fail(
                f"Batch execution timed out after {timeout}s. Not all episodes completed."
            )

        # 3. Verify Per-Episode Isolation (Basic check: different session IDs)
        # We already ensured they have different session IDs and episode IDs.
        # A deeper check would be verifying filesystem isolation, but that is covered by INT-003.
        # Here we just ensure the controller tracked them separately.

        # Double check status one last time
        for episode_id in episode_ids:
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            ep_data = EpisodeResponse.model_validate(status_resp.json())
            assert ep_data.status in [EpisodeStatus.COMPLETED, EpisodeStatus.CANCELLED]
