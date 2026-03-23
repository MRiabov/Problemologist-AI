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
    ConfirmRequest,
    EpisodeResponse,
)
from shared.enums import EpisodeStatus
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import RenderManifest, ReviewManifest
from tests.integration.agent.helpers import repo_git_revision

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
        confirmed = False
        for _ in range(max_retries):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if status_resp.status_code == 404:
                await asyncio.sleep(1)
                continue
            assert status_resp.status_code == 200, status_resp.text
            sess_data = EpisodeResponse.model_validate(status_resp.json())
            status = sess_data.status
            if status == EpisodeStatus.PLANNED and not confirmed:
                await client.post(
                    f"/benchmark/{benchmark_session_id}/confirm",
                    json=ConfirmRequest(comment="Proceed").model_dump(),
                )
                confirmed = True
            elif status == EpisodeStatus.COMPLETED:
                break
            elif status == EpisodeStatus.FAILED:
                pytest.fail(
                    "Benchmark generation failed during setup "
                    f"(session_id={benchmark_session_id})."
                )
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
        episode_assets_resp = await client.get(f"/episodes/{episode_id}")
        assert episode_assets_resp.status_code == 200, (
            f"Failed to fetch episode assets for {episode_id}: {episode_assets_resp.text}"
        )
        episode_data = EpisodeResponse.model_validate(episode_assets_resp.json())
        assert episode_data.metadata_vars is not None
        assert episode_data.metadata_vars.benchmark_id == str(benchmark_session_id)
        assert episode_data.metadata_vars.worker_session_id == engineer_session_id
        assert episode_data.metadata_vars.episode_type == "engineer"
        assert episode_data.metadata_vars.detailed_status in {
            EpisodeStatus.COMPLETED.value,
            EpisodeStatus.FAILED.value,
        }
        assert episode_data.metadata_vars.terminal_reason is not None
        if episode_data.status == EpisodeStatus.FAILED:
            assert episode_data.metadata_vars.failure_class is not None
        else:
            assert episode_data.metadata_vars.failure_class is None
        artifact_paths = [a.s3_path for a in (episode_data.assets or [])]

        benchmark_episode_resp = await client.get(f"/episodes/{benchmark_session_id}")
        assert benchmark_episode_resp.status_code == 200, benchmark_episode_resp.text
        benchmark_episode_data = EpisodeResponse.model_validate(
            benchmark_episode_resp.json()
        )
        benchmark_artifact_paths = [
            a.s3_path for a in (benchmark_episode_data.assets or [])
        ]

        async def _read_episode_asset_text(episode_ref: str, asset_path: str) -> str:
            resp = await client.get(f"/episodes/{episode_ref}/assets/{asset_path}")
            assert resp.status_code == 200, resp.text
            return resp.text

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
        assert any(p.endswith("benchmark_definition.yaml") for p in artifact_paths), (
            f"benchmark_definition.yaml missing. Artifacts: {artifact_paths}"
        )
        assert any(
            p.endswith("benchmark_assembly_definition.yaml") for p in artifact_paths
        ), f"benchmark_assembly_definition.yaml missing. Artifacts: {artifact_paths}"

        source_benchmark_definition_path = next(
            p
            for p in benchmark_artifact_paths
            if p.endswith("benchmark_definition.yaml")
        )
        source_benchmark_assembly_path = next(
            p
            for p in benchmark_artifact_paths
            if p.endswith("benchmark_assembly_definition.yaml")
        )
        source_benchmark_review_manifest_path = next(
            p
            for p in benchmark_artifact_paths
            if p.endswith("benchmark_review_manifest.json")
        )
        source_benchmark_plan_manifest_path = next(
            p
            for p in benchmark_artifact_paths
            if p.endswith("benchmark_plan_review_manifest.json")
        )
        source_render_manifest_path = next(
            p
            for p in benchmark_artifact_paths
            if p.endswith("renders/render_manifest.json")
        )
        engineer_benchmark_definition_path = next(
            p for p in artifact_paths if p.endswith("benchmark_definition.yaml")
        )
        engineer_benchmark_assembly_path = next(
            p
            for p in artifact_paths
            if p.endswith("benchmark_assembly_definition.yaml")
        )
        engineer_benchmark_review_manifest_path = next(
            p for p in artifact_paths if p.endswith("benchmark_review_manifest.json")
        )
        engineer_benchmark_plan_manifest_path = next(
            p
            for p in artifact_paths
            if p.endswith("benchmark_plan_review_manifest.json")
        )
        engineer_render_manifest_path = next(
            p for p in artifact_paths if p.endswith("renders/render_manifest.json")
        )

        source_benchmark_definition_text = await _read_episode_asset_text(
            str(benchmark_session_id), source_benchmark_definition_path
        )
        engineer_benchmark_definition_text = await _read_episode_asset_text(
            episode_id, engineer_benchmark_definition_path
        )
        assert engineer_benchmark_definition_text == source_benchmark_definition_text

        source_benchmark_assembly_text = await _read_episode_asset_text(
            str(benchmark_session_id), source_benchmark_assembly_path
        )
        engineer_benchmark_assembly_text = await _read_episode_asset_text(
            episode_id, engineer_benchmark_assembly_path
        )
        assert engineer_benchmark_assembly_text == source_benchmark_assembly_text

        source_benchmark_review_manifest_text = await _read_episode_asset_text(
            str(benchmark_session_id), source_benchmark_review_manifest_path
        )
        engineer_benchmark_review_manifest_text = await _read_episode_asset_text(
            episode_id, engineer_benchmark_review_manifest_path
        )
        assert (
            engineer_benchmark_review_manifest_text
            == source_benchmark_review_manifest_text
        )

        source_benchmark_plan_manifest_text = await _read_episode_asset_text(
            str(benchmark_session_id), source_benchmark_plan_manifest_path
        )
        engineer_benchmark_plan_manifest_text = await _read_episode_asset_text(
            episode_id, engineer_benchmark_plan_manifest_path
        )
        assert (
            engineer_benchmark_plan_manifest_text == source_benchmark_plan_manifest_text
        )

        source_render_manifest_text = await _read_episode_asset_text(
            str(benchmark_session_id), source_render_manifest_path
        )
        engineer_render_manifest_text = await _read_episode_asset_text(
            episode_id, engineer_render_manifest_path
        )
        assert engineer_render_manifest_text == source_render_manifest_text

        source_benchmark_review_manifest = ReviewManifest.model_validate_json(
            source_benchmark_review_manifest_text
        )
        assert source_benchmark_review_manifest.episode_id == str(
            benchmark_episode_data.id
        )
        assert source_benchmark_review_manifest.worker_session_id == str(
            benchmark_session_id
        )
        assert source_benchmark_review_manifest.benchmark_episode_id == str(
            benchmark_episode_data.id
        )
        assert source_benchmark_review_manifest.benchmark_worker_session_id == str(
            benchmark_session_id
        )
        assert (
            source_benchmark_review_manifest.benchmark_revision == repo_git_revision()
        )
        assert source_benchmark_review_manifest.solution_revision == repo_git_revision()
        benchmark_assembly_definition = yaml.safe_load(source_benchmark_assembly_text)
        assert (
            source_benchmark_review_manifest.environment_version
            == benchmark_assembly_definition["version"]
        )
        assert source_benchmark_review_manifest.preview_evidence_paths
        assert set(source_benchmark_review_manifest.preview_evidence_paths) == set(
            source_benchmark_review_manifest.renders
        )

        source_render_manifest = RenderManifest.model_validate_json(
            source_render_manifest_text
        )
        assert source_render_manifest.episode_id == str(benchmark_episode_data.id)
        assert source_render_manifest.worker_session_id == str(benchmark_session_id)
        assert source_render_manifest.revision == repo_git_revision()
        assert (
            source_render_manifest.environment_version
            == benchmark_assembly_definition["version"]
        )
        assert source_render_manifest.preview_evidence_paths
        assert set(source_render_manifest.preview_evidence_paths) == set(
            source_benchmark_review_manifest.renders
        )

        # Check for Reviewer artifacts
        # Reviews are usually in reviews/ folder
        review_files = [p for p in artifact_paths if "reviews/" in p]
        assert len(review_files) > 0, (
            f"No reviews found (Planner -> Reviewer or Coder -> Reviewer loop missing). Artifacts: {artifact_paths}"
        )
        assert any("validation_results.json" in p for p in artifact_paths), (
            f"validation_results.json missing. Artifacts: {artifact_paths}"
        )
        assert any("simulation_result.json" in p for p in artifact_paths), (
            f"simulation_result.json missing. Artifacts: {artifact_paths}"
        )
        manifest_paths = [
            p
            for p in artifact_paths
            if p.endswith("engineering_execution_review_manifest.json")
        ]
        assert manifest_paths, (
            "engineering_execution_review_manifest.json missing. "
            f"Artifacts: {artifact_paths}"
        )
        assert any(
            "/.manifests/" in p or p.startswith(".manifests/") for p in manifest_paths
        ), f"review manifest must be in .manifests/. Found: {manifest_paths}"
        manifest_resp = await client.get(
            f"/episodes/{episode_id}/assets/{manifest_paths[0]}"
        )
        assert manifest_resp.status_code == 200, manifest_resp.text
        manifest = ReviewManifest.model_validate_json(manifest_resp.text)
        assert manifest.episode_id == str(episode_id)
        assert manifest.worker_session_id == engineer_session_id
        assert manifest.revision == repo_git_revision()
        assert manifest.solution_revision == repo_git_revision()
        assert manifest.environment_version == benchmark_assembly_definition["version"]
        assert manifest.validation_success is True
        assert manifest.simulation_success is True
        assert manifest.goal_reached is True
        assert manifest.preview_evidence_paths
        assert set(manifest.preview_evidence_paths) == set(manifest.renders)
        assert all(render_path in artifact_paths for render_path in manifest.renders)


async def _wait_for_episode_terminal(
    client: AsyncClient,
    episode_id: str,
    *,
    timeout_seconds: int = 240,
) -> EpisodeResponse:
    deadline = asyncio.get_running_loop().time() + timeout_seconds
    last_status = None

    while asyncio.get_running_loop().time() < deadline:
        response = await client.get(f"/episodes/{episode_id}")
        assert response.status_code == 200, response.text
        episode = EpisodeResponse.model_validate(response.json())
        last_status = episode.status
        if episode.status in {
            EpisodeStatus.COMPLETED,
            EpisodeStatus.FAILED,
            EpisodeStatus.CANCELLED,
        }:
            return episode
        await asyncio.sleep(1.0)

    pytest.fail(
        f"Episode {episode_id} did not reach a terminal state (last={last_status})"
    )


async def _reject_episode(client: AsyncClient, episode_id: str) -> EpisodeResponse:
    review_content = """---
decision: rejected
comments: ["Retry lineage test rejection"]
evidence:
  files_checked: ["plan.md"]
---
Rejecting the episode for deterministic retry coverage.
"""
    response = await client.post(
        f"/episodes/{episode_id}/review",
        json={"review_content": review_content},
    )
    assert response.status_code == 200, response.text
    status_response = await client.get(f"/episodes/{episode_id}")
    assert status_response.status_code == 200, status_response.text
    episode = EpisodeResponse.model_validate(status_response.json())
    assert episode.status == EpisodeStatus.FAILED
    return episode


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_engineering_retry_reuses_same_benchmark_linkage():
    """
    INT-205: Retry against the same benchmark must create a distinct engineer
    episode that preserves benchmark linkage and lineage metadata.
    """

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        benchmark_request = BenchmarkGenerateRequest(
            prompt="Create a simple benchmark setup for retry lineage testing.",
            backend=SimulatorBackendType.GENESIS,
        )
        benchmark_resp = await client.post(
            "/benchmark/generate", json=benchmark_request.model_dump()
        )
        assert benchmark_resp.status_code in (200, 202), benchmark_resp.text
        benchmark_session_id = BenchmarkGenerateResponse.model_validate(
            benchmark_resp.json()
        ).session_id

        benchmark_confirmed = False
        benchmark_episode = None
        for _ in range(150):
            status_resp = await client.get(f"/benchmark/{benchmark_session_id}")
            if status_resp.status_code == 404:
                await asyncio.sleep(1.0)
                continue
            assert status_resp.status_code == 200, status_resp.text
            benchmark_episode = EpisodeResponse.model_validate(status_resp.json())
            if (
                benchmark_episode.status == EpisodeStatus.PLANNED
                and not benchmark_confirmed
            ):
                await client.post(
                    f"/benchmark/{benchmark_session_id}/confirm",
                    json=ConfirmRequest(comment="Proceed").model_dump(),
                )
                benchmark_confirmed = True
            elif benchmark_episode.status == EpisodeStatus.COMPLETED:
                break
            elif benchmark_episode.status == EpisodeStatus.FAILED:
                pytest.fail(
                    "Benchmark generation failed during retry setup "
                    f"(session_id={benchmark_session_id})."
                )
            await asyncio.sleep(2)
        else:
            pytest.fail("Benchmark generation failed or timed out during retry setup.")

        original_task = "Create an engineer handoff for retry lineage testing."
        first_session_id = f"INT-205-{uuid.uuid4().hex[:8]}"
        first_run = AgentRunRequest(
            task=original_task,
            session_id=first_session_id,
            metadata_vars={"benchmark_id": str(benchmark_session_id)},
        )
        first_resp = await client.post("/agent/run", json=first_run.model_dump())
        assert first_resp.status_code in (200, 202), first_resp.text
        first_episode_id = str(
            AgentRunResponse.model_validate(first_resp.json()).episode_id
        )

        first_episode = await _wait_for_episode_terminal(client, first_episode_id)
        assert first_episode.metadata_vars is not None
        assert first_episode.metadata_vars.benchmark_id == str(benchmark_session_id)
        assert first_episode.metadata_vars.episode_type == "engineer"
        first_episode = await _reject_episode(client, first_episode_id)

        retry_metadata = {
            "benchmark_id": str(benchmark_session_id),
            "prior_episode_id": first_episode_id,
            "is_reused": True,
        }
        second_session_id = f"INT-205-{uuid.uuid4().hex[:8]}"
        second_run = AgentRunRequest(
            task=original_task,
            session_id=second_session_id,
            metadata_vars=retry_metadata,
        )
        second_resp = await client.post("/agent/run", json=second_run.model_dump())
        assert second_resp.status_code in (200, 202), second_resp.text
        second_episode_id = str(
            AgentRunResponse.model_validate(second_resp.json()).episode_id
        )

        second_episode = await _wait_for_episode_terminal(client, second_episode_id)
        assert second_episode.metadata_vars is not None
        assert second_episode.metadata_vars.benchmark_id == str(benchmark_session_id)
        assert second_episode.metadata_vars.prior_episode_id == first_episode_id
        assert second_episode.metadata_vars.is_reused is True
        assert second_episode.metadata_vars.episode_type == "engineer"
        second_episode = await _reject_episode(client, second_episode_id)

        assert second_episode.id != first_episode.id

        async def _read_asset_text(episode_ref: str, asset_path: str) -> str:
            asset_resp = await client.get(
                f"/episodes/{episode_ref}/assets/{asset_path}"
            )
            assert asset_resp.status_code == 200, asset_resp.text
            return asset_resp.text

        benchmark_definition_path = "benchmark_definition.yaml"
        benchmark_assembly_path = "benchmark_assembly_definition.yaml"
        first_benchmark_definition = await _read_asset_text(
            first_episode_id, benchmark_definition_path
        )
        second_benchmark_definition = await _read_asset_text(
            second_episode_id, benchmark_definition_path
        )
        assert second_benchmark_definition == first_benchmark_definition

        first_benchmark_assembly = await _read_asset_text(
            first_episode_id, benchmark_assembly_path
        )
        second_benchmark_assembly = await _read_asset_text(
            second_episode_id, benchmark_assembly_path
        )
        assert second_benchmark_assembly == first_benchmark_assembly

        assert benchmark_episode is not None
        assert benchmark_episode.metadata_vars is not None
        assert benchmark_episode.metadata_vars.episode_type == "benchmark"
        assert benchmark_episode.status == EpisodeStatus.COMPLETED
