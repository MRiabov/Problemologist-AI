import asyncio

import pytest
from httpx import AsyncClient

from controller.api.schemas import (
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus
from shared.simulation.schemas import SimulatorBackendType

# Adjust URL to your controller if different
CONTROLLER_URL = "http://127.0.0.1:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_benchmark_planner_cad_reviewer_path():
    """
    INT-031: Benchmark planner -> CAD -> reviewer path

    Verifies:
    1. Benchmark generation trigger
    2. Successful completion of the workflow
    3. Existence of required artifacts (plan.md, objectives.yaml, Reviews)
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Trigger Benchmark Generation
        request = BenchmarkGenerateRequest(
            prompt="Create a simple path planning benchmark with a wall and a goal.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to trigger benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = str(benchmark_resp.session_id)

        max_retries = 150
        benchmark_completed = False
        last_status = None
        confirmed = False

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = EpisodeResponse.model_validate(status_resp.json())
                last_status = sess_data.status

                if last_status == EpisodeStatus.PLANNED and not confirmed:
                    # WP08: Call confirm to continue from planning to execution
                    confirm_resp = await client.post(
                        f"/benchmark/{session_id}/confirm",
                        json=ConfirmRequest(comment="Looks good").model_dump(),
                    )
                    assert confirm_resp.status_code in [200, 202]
                    confirmed = True

                if last_status == EpisodeStatus.COMPLETED:
                    benchmark_completed = True
                    break
                if last_status == EpisodeStatus.FAILED:
                    pytest.fail(
                        f"Benchmark generation failed with status: {last_status}"
                    )

            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail(f"Benchmark generation timed out. Last status: {last_status}")

        # 3. Verify Artifacts from episode assets
        episode_resp = await client.get(f"/episodes/{session_id}")
        assert episode_resp.status_code == 200, (
            f"Failed to fetch episode assets: {episode_resp.text}"
        )
        episode_data = EpisodeResponse.model_validate(episode_resp.json())
        artifact_paths = [a.s3_path for a in (episode_data.assets or [])]

        assert any(p.endswith("plan.md") for p in artifact_paths), (
            f"plan.md missing. Artifacts: {artifact_paths}"
        )
        assert any(p.endswith("objectives.yaml") for p in artifact_paths), (
            f"objectives.yaml missing. Artifacts: {artifact_paths}"
        )
        assert any("reviews/" in p for p in artifact_paths) or any(
            p.endswith("validation_results.json") for p in artifact_paths
        ), f"Reviewer/validation outputs missing. Artifacts: {artifact_paths}"
