import asyncio
import uuid

import pytest
from httpx import AsyncClient

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    ArtifactEntry,
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
async def test_engineering_full_loop():
    """
    INT-033: Engineering full loop (planner/coder/reviewer)

    Verifies:
    1. Triggers Engineering Agent on a valid benchmark
    2. Planner generates plan.md (with budgets) and todo.md
    3. Coder attempts implementation (producing script.py or similar)
    4. Reviewer approves/rejects with typed decision

    Note: Requires a valid benchmark session ID.
    If we cannot rely on a pre-existing one, we generate one first.
    """
    async with AsyncClient(
        base_url=CONTROLLER_URL, timeout=300.0
    ) as client:  # Longer timeout for full loop
        # 1. Setup: Generate a Benchmark (or use a mocked ID if testing against mock)
        # For integration, we generate one.
        request = BenchmarkGenerateRequest(
            prompt="Create a benchmark about stacking blocks.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to generate benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        benchmark_session_id = benchmark_resp.session_id

        # Wait for benchmark
        max_retries = 150
        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if status_resp.status_code == 200:
                sess_data = EpisodeResponse.model_validate(status_resp.json())
                status = sess_data.status
                if status == EpisodeStatus.PLANNED:
                    await client.post(
                        f"/benchmark/{benchmark_session_id}/confirm",
                        json=ConfirmRequest(comment="Proceed").model_dump(),
                    )
                elif status == EpisodeStatus.COMPLETED:
                    break
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation failed or timed out during setup.")

        # 2. Trigger Engineer Agent
        engineer_session_id = f"INT-033-{uuid.uuid4().hex[:8]}"
        task = f"Solve benchmark: {benchmark_session_id}"
        run_request = AgentRunRequest(
            task=task,
            session_id=engineer_session_id,
            metadata_vars={"benchmark_id": str(benchmark_session_id)},
        )

        run_resp = await client.post("/agent/run", json=run_request.model_dump())
        assert run_resp.status_code in [
            200,
            202,
        ], f"Failed to trigger agent: {run_resp.text}"
        agent_run_resp = AgentRunResponse.model_validate(run_resp.json())
        episode_id = agent_run_resp.episode_id

        # 3. Poll for Engineering Completion
        engineer_completed = False
        last_status = None

        for _ in range(150):  # Poll for up to 2 mins
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep_data = EpisodeResponse.model_validate(ep_resp.json())
                last_status = ep_data.status
                if last_status in [
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                    "max_turns_reached",
                ]:
                    engineer_completed = True
                    break
            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail(f"Engineer loop timed out. Last status: {last_status}")

        # 4. Verify Engineering Artifacts
        artifacts_resp = await client.get(f"/artifacts/{engineer_session_id}")
        assert artifacts_resp.status_code == 200, (
            f"Failed to fetch artifacts for {engineer_session_id}: {artifacts_resp.text}"
        )
        artifacts = [ArtifactEntry.model_validate(a) for a in artifacts_resp.json()]
        artifact_paths = [a.path for a in artifacts]

        # Check for Planner artifacts
        assert any("plan.md" in p for p in artifact_paths), (
            f"plan.md missing. Artifacts: {artifact_paths}"
        )
        assert any("todo.md" in p for p in artifact_paths), (
            f"todo.md missing. Artifacts: {artifact_paths}"
        )
        assert any("assembly_definition.yaml" in p for p in artifact_paths), (
            f"assembly_definition.yaml missing. Artifacts: {artifact_paths}"
        )

        # Check for Reviewer artifacts
        # Reviews are usually in reviews/ folder
        review_files = [p for p in artifact_paths if "reviews/" in p]
        assert len(review_files) > 0, (
            f"No reviews found (Planner -> Reviewer or Coder -> Reviewer loop missing). Artifacts: {artifact_paths}"
        )
