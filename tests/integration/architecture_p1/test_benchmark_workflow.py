import asyncio

import pytest
from httpx import AsyncClient

from controller.api.schemas import (
    BenchmarkGenerateRequest,
    BenchmarkGenerateResponse,
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import EpisodePhase, EpisodeStatus, TerminalReason
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import ReviewManifest

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
    3. Existence of required artifacts (plan.md, benchmark_definition.yaml, Reviews)
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

        final_status_resp = await client.get(f"/benchmark/{session_id}")
        assert final_status_resp.status_code == 200, final_status_resp.text
        final_episode = EpisodeResponse.model_validate(final_status_resp.json())
        final_metadata = final_episode.metadata_vars
        assert final_metadata is not None, "Episode metadata is missing."
        assert final_metadata.terminal_reason == TerminalReason.APPROVED
        assert final_metadata.failure_class is None
        assert final_metadata.episode_phase == EpisodePhase.BENCHMARK_REVIEWING

        # 3. Verify Artifacts from episode assets
        episode_resp = await client.get(f"/episodes/{session_id}")
        assert episode_resp.status_code == 200, (
            f"Failed to fetch episode assets: {episode_resp.text}"
        )
        episode_data = EpisodeResponse.model_validate(episode_resp.json())
        artifact_paths = [a.s3_path for a in (episode_data.assets or [])]
        traces = episode_data.traces or []
        submit_plan_traces = [
            t
            for t in traces
            if t.trace_type.value == "TOOL_START" and t.name == "submit_plan"
        ]
        inspect_media_traces = [
            t
            for t in traces
            if t.trace_type.value == "TOOL_START" and t.name == "inspect_media"
        ]

        assert any(p.endswith("plan.md") for p in artifact_paths), (
            f"plan.md missing. Artifacts: {artifact_paths}"
        )
        assert any(p.endswith("benchmark_definition.yaml") for p in artifact_paths), (
            f"benchmark_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assert any(p.endswith("assembly_definition.yaml") for p in artifact_paths), (
            f"assembly_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assert submit_plan_traces, (
            "Expected planner to call submit_plan before workflow completion."
        )
        assert inspect_media_traces, (
            "Expected benchmark reviewer to inspect a render before approval."
        )
        assert any(t.name == "media_inspection" for t in traces), (
            "media_inspection event missing from benchmark reviewer run."
        )
        assert any(t.name == "llm_media_attached" for t in traces), (
            "llm_media_attached event missing from benchmark reviewer run."
        )
        assert any(p.endswith("validation_results.json") for p in artifact_paths), (
            f"validation_results.json missing. Artifacts: {artifact_paths}"
        )
        assert any(p.endswith("simulation_result.json") for p in artifact_paths), (
            f"simulation_result.json missing. Artifacts: {artifact_paths}"
        )
        manifest_paths = [
            p
            for p in artifact_paths
            if p.endswith("benchmark_plan_review_manifest.json")
        ]
        assert manifest_paths, (
            f"benchmark_plan_review_manifest.json missing. Artifacts: {artifact_paths}"
        )
        assert any(
            p.endswith("benchmark-plan-review-round-1.md") for p in artifact_paths
        ), (
            "benchmark plan review file missing from artifacts. "
            f"Artifacts: {artifact_paths}"
        )
        manifest_paths = [
            p for p in artifact_paths if p.endswith("benchmark_review_manifest.json")
        ]
        assert manifest_paths, (
            f"benchmark_review_manifest.json missing. Artifacts: {artifact_paths}"
        )
        assert not any(
            p.endswith("engineering_execution_review_manifest.json")
            for p in artifact_paths
        ), (
            "Benchmark workflow must not emit engineering_execution_review_manifest.json. "
            f"Artifacts: {artifact_paths}"
        )
        assert any(
            "/.manifests/" in p or p.startswith(".manifests/") for p in manifest_paths
        ), f"review manifest must be in .manifests/. Found: {manifest_paths}"
        manifest_resp = await client.get(
            f"/episodes/{session_id}/assets/{manifest_paths[0]}"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = ReviewManifest.model_validate_json(manifest_resp.text)
        assert manifest.status == "ready_for_review"
        assert manifest.validation_success is True
        assert manifest.simulation_success is True
        assert manifest.goal_reached is True

        script_paths = [p for p in artifact_paths if p.endswith("script.py")]
        assert script_paths, f"script.py missing. Artifacts: {artifact_paths}"
        script_resp = await client.get(
            f"/episodes/{session_id}/assets/{script_paths[0]}"
        )
        assert script_resp.status_code == 200, script_resp.text
        assert "__main__" not in script_resp.text
        assert "from utils.metadata import" in script_resp.text
        assert "from utils.submission import" in script_resp.text
