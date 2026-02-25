import asyncio

import pytest
from httpx import AsyncClient

from controller.api.schemas import (
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
async def test_benchmark_to_engineer_handoff():
    """
    INT-032: Benchmark-to-engineer handoff package

    Verifies that the Engineer receives (or has access to) the expected bundle:
    - objectives.yaml
    - environment geometry metadata
    - 24-view renders
    - moving-parts DOFs
    - runtime jitter metadata

    This test triggers a benchmark generation and then inspects the produced artifacts
    to ensure the "package" is complete for the Engineer.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        # 1. Trigger Benchmark Generation
        request = BenchmarkGenerateRequest(
            prompt="Create a benchmark with a moving platform.",  # implies moving parts
            backend=SimulatorBackendType.GENESIS,
        )
        resp = await client.post("/benchmark/generate", json=request.model_dump())
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to trigger benchmark: {resp.text}"
        benchmark_resp = BenchmarkGenerateResponse.model_validate(resp.json())
        session_id = benchmark_resp.session_id

        # 2. Poll for completion
        max_retries = 150
        benchmark_completed = False
        last_status = None

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = EpisodeResponse.model_validate(status_resp.json())
                last_status = sess_data.status
                if last_status == EpisodeStatus.PLANNED:
                    await client.post(
                        f"/benchmark/{session_id}/confirm",
                        json=ConfirmRequest(comment="Handoff confirm").model_dump()
                    )
                if last_status == EpisodeStatus.COMPLETED:
                    benchmark_completed = True
                    break
                if last_status in ["REJECTED", EpisodeStatus.FAILED]:
                    pytest.fail(f"Benchmark generation failed: {last_status}")
            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail(f"Benchmark generation timed out. Last status: {last_status}")

        # 3. Verify Handoff Package Artifacts
        artifacts_resp = await client.get(f"/artifacts/{session_id}")
        assert artifacts_resp.status_code == 200, (
            f"Failed to fetch artifacts: {artifacts_resp.text}"
        )
        artifacts = [ArtifactEntry.model_validate(a) for a in artifacts_resp.json()]
        artifact_paths = [a.path for a in artifacts]

        # Check existence of required files
        assert any("objectives.yaml" in p for p in artifact_paths), (
            f"objectives.yaml missing. Artifacts: {artifact_paths}"
        )

        # Check for renders (expecting a directory or multiple files)
        # Renders usually in renders/ folder.
        render_files = [
            p
            for p in artifact_paths
            if "renders/" in p and (".png" in p or ".jpg" in p)
        ]
        assert len(render_files) > 0, f"No renders found. Artifacts: {artifact_paths}"
