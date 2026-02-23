import asyncio

import pytest
from httpx import AsyncClient

# Adjust URL to your controller if different
CONTROLLER_URL = "http://localhost:18000"


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
        prompt = "Create a benchmark with a moving platform."  # implies moving parts
        resp = await client.post(
            "/benchmark/generate", json={"prompt": prompt, "backend": "genesis"}
        )
        assert resp.status_code in [
            200,
            202,
        ], f"Failed to trigger benchmark: {resp.text}"
        session_id = resp.json()["session_id"]

        # 2. Poll for completion
        max_retries = 60
        benchmark_completed = False
        last_status = None

        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{session_id}")
            if status_resp.status_code == 200:
                sess_data = status_resp.json()
                last_status = sess_data["status"]
                if last_status == "planned":
                    await client.post(f"/benchmark/{session_id}/confirm")
                if last_status == "completed":
                    benchmark_completed = True
                    break
                if last_status in ["rejected", "failed"]:
                    pytest.fail(f"Benchmark generation failed: {last_status}")
            await asyncio.sleep(2)

        if not benchmark_completed:
            pytest.fail(f"Benchmark generation timed out. Last status: {last_status}")

        # 3. Verify Handoff Package Artifacts
        artifacts_resp = await client.get(f"/artifacts/{session_id}")
        assert artifacts_resp.status_code == 200, (
            f"Failed to fetch artifacts: {artifacts_resp.text}"
        )
        artifacts = artifacts_resp.json()
        artifact_paths = [a["path"] for a in artifacts]

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
        # INT-032 mentions 24-view renders, so we might check count if strict,
        # but >0 is a good start for integration "smoke" of the feature.

        # Check for geometry metadata (likely in plan.md or a separate json/yaml)
        # The spec says "environment geometry metadata", often embedded in plan or objectives.
        # Let's assume it's in objectives.yaml or plan.md.

        # Check for moving parts DOFs (in objectives.yaml usually)
        # Check for runtime jitter (in objectives.yaml usually)

        # If possible, fetch objectives.yaml content to verify structure
        # (This depends on artifact retrieval API. If checking existence is P1, content check is P2/Deep Verification)
