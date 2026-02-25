import asyncio
import uuid

import pytest
from httpx import AsyncClient

from controller.api.routes.benchmark import BenchmarkGenerateRequest
from controller.api.schemas import (
    AgentRunResponse,
    BenchmarkGenerateResponse,
    EpisodeResponse,
)
from controller.api.tasks import AgentRunRequest
from shared.enums import EpisodeStatus

# Adjust URL to your controller
CONTROLLER_URL = "http://localhost:18000"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_full_workflow_end_to_end():
    """
    Test the full workflow:
    1. Trigger Benchmark Generator (via API)
    2. Wait for Benchmark to complete and produce assets
    3. Trigger Engineer Agent (via API) with the generated benchmark
    4. Wait for Engineer to complete

    We mock the LLM responses to ensure deterministic execution and avoid costs.
    """

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=60.0) as client:
        # 1. Trigger Benchmark Generation
        prompt = "Create a simple box stacking benchmark."
        req = BenchmarkGenerateRequest(prompt=prompt)
        resp = await client.post("/benchmark/generate", json=req.model_dump())
        assert resp.status_code == 200, f"Failed to trigger benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = benchmark_resp.session_id

        # 2. Poll for completion
        max_retries = 30
        benchmark_completed = False
        last_benchmark_status = None

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = EpisodeResponse.model_validate(status_resp.json())
                last_benchmark_status = sess_data.status

                if last_benchmark_status == EpisodeStatus.COMPLETED:
                    benchmark_completed = True
                    break
                if (
                    last_benchmark_status == "REJECTED"
                    or last_benchmark_status == EpisodeStatus.FAILED
                ):
                    pytest.fail("Benchmark generation was rejected.")

            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail(
                f"Benchmark generation timed out. Last status: {last_benchmark_status}"
            )

        engineer_task = (
            f"Solve this benchmark: {session_id}. Constraints: Use standard parts."
        )

        # 4. Trigger Engineer Agent
        # Generate a unique session ID for the engineer run
        engineer_session_id = f"INT-033-full-{uuid.uuid4().hex[:8]}"

        req_run = AgentRunRequest(task=engineer_task, session_id=engineer_session_id)

        run_resp = await client.post("/agent/run", json=req_run.model_dump())
        assert run_resp.status_code == 202, (
            f"Failed to trigger engineer: {run_resp.text}"
        )
        agent_run_resp = AgentRunResponse.model_validate(run_resp.json())
        episode_id = agent_run_resp.episode_id

        # 5. Poll for Engineer completion
        engineer_completed = False
        last_engineer_status = None
        for _ in range(max_retries):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep_data = EpisodeResponse.model_validate(ep_resp.json())
                last_engineer_status = ep_data.status

                if last_engineer_status == EpisodeStatus.COMPLETED:
                    engineer_completed = True
                    break
                if last_engineer_status == EpisodeStatus.FAILED:
                    pytest.fail(f"Engineer agent failed. Episode data: {ep_data}")

            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail(
                f"Engineer agent timed out. Last status: {last_engineer_status}"
            )
