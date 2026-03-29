import asyncio
import hashlib
import time
import uuid
from pathlib import Path

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
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, FailureReason, ReviewDecision
from shared.models.simulation import (
    MultiRunResult,
    SimulationFailure,
    SimulationMetrics,
    SimulationResult,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.schema import (
    ValidationResultRecord,
    WriteFileRequest,
)
from tests.integration.agent.helpers import wait_for_episode_terminal

# Adjust URL to your controller if different
CONTROLLER_URL = "http://127.0.0.1:18000"


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_engineering_full_loop():
    """
    INT-033: Engineering execution reviewer stability review.

    Seeds a reviewable engineer episode directly in the controller database,
    then submits a stability review against the live review endpoint so the
    persisted stability summary, review decision trail, and render evidence are
    validated without depending on the slower planning/coding path.
    """
    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        episode_id = uuid.uuid4()
        worker_session_id = f"INT-033-{uuid.uuid4().hex[:8]}"
        task = f"Review peer solution stability: {worker_session_id}"

        cad_preview_bytes = Path(
            "tests/integration/mock_responses/INT-039/engineer_coder/entry_01/02__renders_preview.png"
        ).read_bytes()
        simulation_preview_bytes = cad_preview_bytes
        render_probe_bytes = cad_preview_bytes
        verification_result = MultiRunResult(
            num_scenes=4,
            success_count=3,
            success_rate=0.75,
            is_consistent=False,
            individual_results=[
                SimulationMetrics(success=True),
                SimulationMetrics(success=True),
                SimulationMetrics(success=True),
                SimulationMetrics(
                    success=False,
                    fail_reason="Scene 4 lost balance under jitter",
                    fail_mode=FailureReason.STABILITY_ISSUE,
                    failure=SimulationFailure(
                        reason=FailureReason.STABILITY_ISSUE,
                        detail="Scene 4 drifted off the support plane.",
                    ),
                ),
            ],
            fail_reasons=["Scene 4 lost balance under jitter"],
            scene_build_count=1,
            backend_run_count=1,
            batched_execution=True,
        )
        review_timestamp = time.time()
        stability_summary = {
            "batchWidth": verification_result.num_scenes,
            "successCount": verification_result.success_count,
            "successRate": verification_result.success_rate,
            "isConsistent": verification_result.is_consistent,
            "sceneBuildCount": verification_result.scene_build_count,
            "backendRunCount": verification_result.backend_run_count,
            "batchedExecution": verification_result.batched_execution,
            "sceneSummaries": [
                {
                    "sceneIndex": idx + 1,
                    "success": scene.success,
                    "summary": (
                        f"Scene {idx + 1}: pass"
                        if scene.success
                        else f"Scene {idx + 1}: {scene.fail_reason}"
                    ),
                    "failReason": scene.fail_reason,
                    "failureMode": scene.fail_mode.value if scene.fail_mode else None,
                }
                for idx, scene in enumerate(verification_result.individual_results)
            ],
        }
        review_content = (
            "---\n"
            + yaml.safe_dump(
                {
                    "decision": "approved",
                    "comments": [
                        "Stable under runtime jitter.",
                    ],
                    "evidence": {
                        "stability_summary": stability_summary,
                        "stability_summary_source": "validation_results.json",
                        "review_context": {
                            "episode_id": str(episode_id),
                            "worker_session_id": worker_session_id,
                        },
                    },
                },
                sort_keys=False,
            ).strip()
            + "\n---\n"
            + "\n".join(
                [
                    "# Peer Review",
                    "",
                    "Stability summary:",
                    "- Source artifact: validation_results.json",
                    "- Batch width: 4",
                    "- Success count: 3 / 4",
                    "- Success rate: 75.0%",
                    "- Consistency: inconsistent",
                    "- Batched execution: yes",
                ]
            )
            + "\n"
        )

        async def _write_workspace_file(path: str, content: str) -> None:
            response = await client.post(
                "http://127.0.0.1:18001/fs/write",
                json=WriteFileRequest(
                    path=path,
                    content=content,
                    overwrite=True,
                    bypass_agent_permissions=True,
                ).model_dump(),
                headers={
                    "X-Session-ID": worker_session_id,
                    "X-System-FS-Bypass": "1",
                },
            )
            assert response.status_code == 200, response.text

        async def _upload_workspace_file(
            path: str, content: bytes, content_type: str
        ) -> None:
            response = await client.post(
                "http://127.0.0.1:18001/fs/upload_file",
                data={
                    "path": path,
                    "bypass_agent_permissions": "true",
                },
                files={
                    "file": (
                        Path(path).name,
                        content,
                        content_type,
                    )
                },
                headers={
                    "X-Session-ID": worker_session_id,
                    "X-System-FS-Bypass": "1",
                },
            )
            assert response.status_code == 200, response.text

        await _write_workspace_file(
            "script.py",
            "print('peer review stable')\n",
        )
        await _write_workspace_file(
            "validation_results.json",
            ValidationResultRecord(
                success=True,
                message="Validation completed with batched jitter verification.",
                timestamp=review_timestamp,
                script_path="script.py",
                script_sha256=hashlib.sha256(
                    b"print('peer review stable')\n"
                ).hexdigest(),
                verification_result=verification_result,
            ).model_dump_json(indent=2),
        )
        await _write_workspace_file(
            "simulation_result.json",
            SimulationResult(
                success=True,
                summary="Goal achieved in green zone.",
                render_paths=[
                    "renders/render_e45_a45.png",
                    "renders/cad_preview.png",
                    "renders/simulation_preview.png",
                ],
                confidence="high",
            ).model_dump_json(indent=2),
        )
        await _upload_workspace_file(
            "renders/render_e45_a45.png", render_probe_bytes, "image/png"
        )
        await _upload_workspace_file(
            "renders/cad_preview.png", cad_preview_bytes, "image/png"
        )
        await _upload_workspace_file(
            "renders/simulation_preview.png", simulation_preview_bytes, "image/png"
        )

        session_factory = get_sessionmaker()
        async with session_factory() as db:
            db.add(
                Episode(
                    id=episode_id,
                    task=task,
                    status=EpisodeStatus.RUNNING,
                    metadata_vars={
                        "worker_session_id": worker_session_id,
                        "episode_type": "engineer",
                    },
                )
            )
            await db.commit()

        review_resp = await client.post(
            f"/episodes/{episode_id}/review",
            json={"review_content": review_content},
        )
        assert review_resp.status_code == 200, review_resp.text

        episode_assets_resp = await client.get(f"/episodes/{episode_id}")
        assert episode_assets_resp.status_code == 200, (
            f"Failed to fetch episode assets for {episode_id}: {episode_assets_resp.text}"
        )
        episode_data = EpisodeResponse.model_validate(episode_assets_resp.json())
        assert episode_data.status == EpisodeStatus.COMPLETED
        assert episode_data.metadata_vars is not None
        assert episode_data.metadata_vars.worker_session_id == worker_session_id
        assert episode_data.metadata_vars.episode_type == "engineer"

        async def _read_episode_asset_text(episode_ref: str, asset_path: str) -> str:
            resp = await client.get(f"/episodes/{episode_ref}/assets/{asset_path}")
            assert resp.status_code == 200, resp.text
            return resp.text

        validation_results_text = await _read_episode_asset_text(
            str(episode_id), "validation_results.json"
        )
        validation_results = ValidationResultRecord.model_validate_json(
            validation_results_text
        )
        assert validation_results.verification_result is not None
        assert validation_results.verification_result.num_scenes == 4
        assert validation_results.verification_result.success_count == 3
        assert validation_results.verification_result.scene_build_count == 1
        assert validation_results.verification_result.backend_run_count == 1
        assert validation_results.verification_result.success_rate == 0.75
        assert validation_results.verification_result.is_consistent is False

        simulation_result_text = await _read_episode_asset_text(
            str(episode_id), "simulation_result.json"
        )
        simulation_result = SimulationResult.model_validate_json(simulation_result_text)
        assert "renders/render_e45_a45.png" in simulation_result.render_paths
        assert "renders/cad_preview.png" in simulation_result.render_paths
        assert "renders/simulation_preview.png" in simulation_result.render_paths

        review_traces = [
            trace
            for trace in (episode_data.traces or [])
            if trace.name == "review_decision"
        ]
        assert review_traces, "Expected review_decision trace trail from live review."
        latest_review_trace = max(review_traces, key=lambda trace: trace.id)
        assert latest_review_trace.metadata_vars is not None
        assert latest_review_trace.metadata_vars.decision == ReviewDecision.APPROVED
        assert latest_review_trace.metadata_vars.review_id is not None


async def _wait_for_episode_terminal(
    client: AsyncClient,
    episode_id: str,
    *,
    timeout_seconds: int = 240,
) -> EpisodeResponse:
    return EpisodeResponse.model_validate(
        await wait_for_episode_terminal(
            client,
            episode_id,
            timeout_s=timeout_seconds,
            terminal_statuses={
                EpisodeStatus.COMPLETED,
                EpisodeStatus.FAILED,
                EpisodeStatus.CANCELLED,
            },
        )
    )


async def _reject_episode(client: AsyncClient, episode_id: str) -> EpisodeResponse:
    review_content = """---
decision: rejected
comments: ["Retry lineage test rejection"]
evidence:
  stability_summary:
    source: validation_results.json
    verdict: rejected_for_retry
    note: Intentional rejection for retry lineage coverage.
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


async def _read_episode_asset_text(
    client: AsyncClient, episode_id: str, asset_path: str
) -> str:
    resp = await client.get(f"/episodes/{episode_id}/assets/{asset_path}")
    assert resp.status_code == 200, resp.text
    return resp.text


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
