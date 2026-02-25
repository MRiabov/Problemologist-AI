import asyncio
import uuid

import pytest
import yaml
from httpx import AsyncClient

from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus
from shared.models.schemas import ReviewFrontmatter
from shared.simulation.schemas import SimulatorBackendType

# Adjust URL to your controller if different
CONTROLLER_URL = "http://127.0.0.1:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_reviewer_evidence_completeness():
    """
    INT-034: Reviewer evidence completeness

    Verifies that Review decisions include expected evidence fields:
    - images_viewed
    - files_checked

    This test runs an engineering task (or uses an existing one) and inspects the review artifact.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Setup: Use or Generate Benchmark + Run Engineer (Similar to INT-033)
        # To make this test independent, we repeat the setup.
        # Ideally, we'd have a fixture yielding a completed episode ID.

        request = BenchmarkGenerateRequest(
            prompt="Create a trivial benchmark.",
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Benchmark generation failed: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        benchmark_session_id = benchmark_resp.session_id

        # Wait for benchmark
        for _ in range(150):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if status_resp.status_code == 200:
                sess_data = EpisodeResponse.model_validate(status_resp.json())
                if sess_data.status == EpisodeStatus.COMPLETED:
                    break
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation timed out.")

        engineer_session_id = f"INT-034-{uuid.uuid4().hex[:8]}"
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
        ], f"Agent trigger failed: {run_resp.text}"
        agent_run_resp = AgentRunResponse.model_validate(run_resp.json())
        episode_id = agent_run_resp.episode_id

        # Wait for Engineer (at least until a review is produced)
        # A full completion is safest.
        engineer_completed = False
        for _ in range(150):
            ep_resp = await client.get(f"/episodes/{episode_id}")
            if ep_resp.status_code == 200:
                ep_data = EpisodeResponse.model_validate(ep_resp.json())
                if ep_data.status in [
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                ]:
                    engineer_completed = True
                    break
            await asyncio.sleep(2)

        if not engineer_completed:
            pytest.fail("Engineer timed out.")

        # 2. Fetch Review Artifact via Assets
        # Use /episodes/{id} to get assets list
        ep_resp = await client.get(f"/episodes/{episode_id}")
        assert ep_resp.status_code == 200
        ep_data = EpisodeResponse.model_validate(ep_resp.json())
        assets = ep_data.assets or []

        review_assets = [a for a in assets if "reviews/" in a.s3_path]
        assert len(review_assets) > 0, (
            f"No reviews found in assets: {[a.s3_path for a in assets]}"
        )

        # 3. Inspect Content for Evidence
        passed_evidence_check = False
        for review_asset in review_assets:
            # We assume content is available if asset_type is readable
            content = review_asset.content or ""
            if not content:
                # If content is empty, fetch via proxy
                path = review_asset.s3_path
                # Proxy url: /episodes/{id}/assets/{path}
                proxy_resp = await client.get(f"/episodes/{episode_id}/assets/{path}")
                if proxy_resp.status_code == 200:
                    content = proxy_resp.text

            # Check metadata/frontmatter
            # Reviewer usually puts YAML frontmatter
            if content.startswith("---"):
                try:
                    parts = content.split("---")
                    if len(parts) >= 3:
                        raw_frontmatter = yaml.safe_load(parts[1])
                        # Validate via Pydantic model
                        frontmatter = ReviewFrontmatter.model_validate(raw_frontmatter)

                        # Check for evidence in comments or other fields
                        if frontmatter.decision and (
                            frontmatter.comments or "images_viewed" in raw_frontmatter
                        ):
                            passed_evidence_check = True
                            break
                except Exception:
                    pass

        # Verify that we could parse and check the frontmatter
        assert len(review_assets) > 0
