import hashlib
import time
import uuid
from pathlib import Path

import pytest
import yaml
from httpx import AsyncClient

from controller.api.schemas import EpisodeReplayResponse
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode, Trace
from shared.enums import (
    AgentName,
    AssetType,
    EntryFailureDisposition,
    EpisodeStatus,
    EpisodeType,
    FailureClass,
    FailureReason,
    ReviewDecision,
    TerminalReason,
    TraceType,
)
from shared.git_utils import repo_revision
from shared.models.schemas import (
    EntryValidationContext,
    EntryValidationError,
    EpisodeMetadata,
    TraceMetadata,
)
from shared.models.simulation import (
    MultiRunResult,
    SimulationFailure,
    SimulationMetrics,
    SimulationResult,
)
from shared.workers.schema import (
    DeleteFileRequest,
    ReviewManifest,
    ValidationResultRecord,
    WriteFileRequest,
)

CONTROLLER_URL = "http://127.0.0.1:18000"
WORKER_LIGHT_URL = "http://127.0.0.1:18001"


async def _write_workspace_file(
    client: AsyncClient,
    *,
    session_id: str,
    path: str,
    content: str,
) -> None:
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/write",
        json=WriteFileRequest(
            path=path,
            content=content,
            overwrite=True,
            bypass_agent_permissions=True,
        ).model_dump(),
        headers={
            "X-Session-ID": session_id,
            "X-System-FS-Bypass": "1",
        },
    )
    assert resp.status_code == 200, resp.text


async def _delete_workspace_file(
    client: AsyncClient,
    *,
    session_id: str,
    path: str,
) -> None:
    resp = await client.post(
        f"{WORKER_LIGHT_URL}/fs/delete",
        json=DeleteFileRequest(
            path=path,
            bypass_agent_permissions=True,
        ).model_dump(),
        headers={
            "X-Session-ID": session_id,
            "X-System-FS-Bypass": "1",
        },
    )
    assert resp.status_code == 200, resp.text


async def _seed_replay_episode(
    client: AsyncClient,
    *,
    include_validation_artifact: bool = True,
    manifest_revision: str | None = None,
    unsupported_mechanism: str | None = None,
) -> tuple[str, str, str]:
    episode_id = uuid.uuid4()
    user_session_id = uuid.uuid4()
    worker_session_id = f"INT-206-{uuid.uuid4().hex[:8]}"
    current_revision = repo_revision(Path(__file__).resolve().parents[3])
    assert current_revision, "repository revision unavailable"
    manifest_revision = manifest_revision or current_revision

    script_content = "print('replay seed')\n"
    script_sha256 = hashlib.sha256(script_content.encode("utf-8")).hexdigest()
    seed_ts = time.time()

    validation_record = ValidationResultRecord(
        success=True,
        message="Validation completed before failure.",
        timestamp=seed_ts,
        script_path="script.py",
        script_sha256=script_sha256,
        verification_result=MultiRunResult(
            num_scenes=1,
            success_count=1,
            success_rate=1.0,
            is_consistent=True,
            individual_results=[SimulationMetrics(success=True)],
            fail_reasons=[],
            scene_build_count=1,
            backend_run_count=1,
            batched_execution=True,
        ),
    )
    simulation_result = SimulationResult(
        success=False,
        summary="Simulation failed because the object left the goal corridor.",
        failure_reason=FailureReason.OUT_OF_BOUNDS,
        fail_mode=FailureReason.OUT_OF_BOUNDS,
        failure=SimulationFailure(
            reason=FailureReason.OUT_OF_BOUNDS,
            detail="The object drifted outside the legal replay corridor.",
        ),
        render_paths=["renders/render_e45_a45.png"],
        confidence="high",
    )
    review_manifest = ReviewManifest(
        status="ready_for_review",
        reviewer_stage="engineering_execution_reviewer",
        timestamp=str(seed_ts),
        session_id=worker_session_id,
        revision=manifest_revision,
        episode_id=str(episode_id),
        worker_session_id=worker_session_id,
        benchmark_episode_id=str(episode_id),
        benchmark_worker_session_id=worker_session_id,
        benchmark_revision=manifest_revision,
        solution_revision=manifest_revision,
        environment_version="replay-test",
        preview_evidence_paths=["renders/render_e45_a45.png"],
        script_path="script.py",
        script_sha256=script_sha256,
        validation_success=True,
        validation_timestamp=seed_ts,
        simulation_success=False,
        simulation_summary="Simulation failed because the object left the goal corridor.",
        simulation_timestamp=seed_ts,
        goal_reached=False,
        renders=["renders/render_e45_a45.png"],
        mjcf_path="renders/scene.xml",
        cad_path="renders/model.step",
        objectives_path="renders/benchmark_definition.yaml",
        assembly_definition_path="renders/assembly_definition.yaml",
    )
    entry_validation = EntryValidationContext(
        node=AgentName.ENGINEER_CODER,
        disposition=EntryFailureDisposition.FAIL_FAST,
        reason_code="unsupported_validation_preview",
        reroute_target=AgentName.ENGINEER_PLANNER,
        errors=[
            EntryValidationError(
                code="unsupported_validation_preview",
                message="Persisted entry validation rejected the current path.",
                source="policy",
                artifact_path="benchmark_definition.yaml",
            )
        ],
    )

    episode_metadata = EpisodeMetadata(
        worker_session_id=worker_session_id,
        episode_type=EpisodeType.ENGINEER,
        detailed_status=EpisodeStatus.FAILED.value,
        terminal_reason=TerminalReason.REJECTED_BY_REVIEW,
        failure_class=FailureClass.AGENT_QUALITY_FAILURE,
        validation_logs=[
            "Replay seed reached terminal failure state.",
        ],
        additional_info={
            "entry_validation": entry_validation.model_dump(mode="json"),
        },
    )
    if unsupported_mechanism is not None:
        episode_metadata.additional_info["unsupported_mechanism"] = (
            unsupported_mechanism
        )

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = Episode(
            id=episode_id,
            user_session_id=user_session_id,
            task="Replay a failed episode from persisted artifacts.",
            status=EpisodeStatus.FAILED,
            metadata_vars=episode_metadata.model_dump(mode="json"),
            plan="# Replay Plan\n- Seeded plan for replay coverage.\n",
            journal="# Replay Journal\n- Failure state is persisted.\n",
            todo_list={"completed": False},
        )
        db.add(episode)

        async def _add_asset(path: str, content: str, asset_type: AssetType) -> None:
            db.add(
                Asset(
                    episode_id=episode_id,
                    user_session_id=user_session_id,
                    asset_type=asset_type,
                    s3_path=path,
                    content=content,
                )
            )

        await _add_asset("script.py", script_content, AssetType.PYTHON)
        await _add_asset("plan.md", episode.plan, AssetType.MARKDOWN)
        await _add_asset("journal.md", episode.journal, AssetType.MARKDOWN)
        await _add_asset("todo.md", "completed: false\n", AssetType.MARKDOWN)
        if include_validation_artifact:
            await _add_asset(
                "validation_results.json",
                validation_record.model_dump_json(indent=2),
                AssetType.OTHER,
            )
        await _add_asset(
            "simulation_result.json",
            simulation_result.model_dump_json(indent=2),
            AssetType.OTHER,
        )
        await _add_asset(
            ".manifests/engineering_execution_handoff_manifest.json",
            review_manifest.model_dump_json(indent=2),
            AssetType.OTHER,
        )
        await _add_asset(
            "reviews/engineering-execution-review-decision-round-1.yaml",
            yaml.safe_dump(
                {
                    "decision": "rejected",
                    "comments": ["Replay bundle rejected the run."],
                },
                sort_keys=False,
            ),
            AssetType.OTHER,
        )
        await _add_asset(
            "reviews/engineering-execution-review-comments-round-1.yaml",
            yaml.safe_dump(
                {
                    "summary": "REJECTED: replayable failure state",
                    "checklist": {
                        "stability": "fail",
                        "validation_replayed": "pass",
                    },
                },
                sort_keys=False,
            ),
            AssetType.OTHER,
        )

        db.add(
            Trace(
                episode_id=episode_id,
                user_session_id=user_session_id,
                trace_type=TraceType.EVENT,
                name="node_entry_validation_failed",
                content="ENTRY_VALIDATION_FAILED[unsupported_validation_preview]",
                simulation_run_id="sim-001",
                review_id="review-001",
                metadata_vars=TraceMetadata(
                    simulation_run_id="sim-001",
                    review_id="review-001",
                    observation="entry validation rejected the path",
                    additional_info={
                        "source": "node_entry_validation",
                    },
                ).model_dump(mode="json"),
            )
        )
        db.add(
            Trace(
                episode_id=episode_id,
                user_session_id=user_session_id,
                trace_type=TraceType.EVENT,
                name="simulation_result",
                content="simulation_result.json persisted",
                simulation_run_id="sim-001",
                cots_query_id="cots-001",
                metadata_vars=TraceMetadata(
                    simulation_run_id="sim-001",
                    cots_query_id="cots-001",
                    observation="simulation completed with a deterministic failure",
                ).model_dump(mode="json"),
            )
        )
        db.add(
            Trace(
                episode_id=episode_id,
                user_session_id=user_session_id,
                trace_type=TraceType.EVENT,
                name="review_decision",
                content="Rejected because the replay evidence is deterministic.",
                review_id="review-001",
                metadata_vars=TraceMetadata(
                    review_id="review-001",
                    decision=ReviewDecision.REJECTED,
                    checklist={"replayable_failure": "pass"},
                    observation="review decision persisted",
                ).model_dump(mode="json"),
            )
        )
        await db.commit()

    for path, content in {
        "script.py": script_content,
        "plan.md": episode.plan,
        "journal.md": episode.journal,
        "todo.md": "completed: false\n",
        ".manifests/engineering_execution_handoff_manifest.json": review_manifest.model_dump_json(
            indent=2
        ),
        "reviews/engineering-execution-review-decision-round-1.yaml": yaml.safe_dump(
            {
                "decision": "rejected",
                "comments": ["Replay bundle rejected the run."],
            },
            sort_keys=False,
        ),
        "reviews/engineering-execution-review-comments-round-1.yaml": yaml.safe_dump(
            {
                "summary": "REJECTED: replayable failure state",
                "checklist": {
                    "stability": "fail",
                    "validation_replayed": "pass",
                },
            },
            sort_keys=False,
        ),
    }.items():
        await _write_workspace_file(
            client, session_id=worker_session_id, path=path, content=content
        )
    if include_validation_artifact:
        await _write_workspace_file(
            client,
            session_id=worker_session_id,
            path="validation_results.json",
            content=validation_record.model_dump_json(indent=2),
        )
    await _write_workspace_file(
        client,
        session_id=worker_session_id,
        path="simulation_result.json",
        content=simulation_result.model_dump_json(indent=2),
    )

    return str(episode_id), worker_session_id, str(user_session_id)


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_failed_episode_replay_bundle_contract():
    """
    INT-206: failed episode replay bundle contract.

    Verifies replay reconstructs a failed episode from persisted artifacts only,
    surfaces deterministic failure metadata and trace IDs, stays stable across
    repeated calls, and fails closed on missing/stale/unsupported replay evidence.
    """

    async with AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        episode_id, worker_session_id, user_session_id = await _seed_replay_episode(
            client
        )

        replay_resp = await client.get(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/replay"
        )
        assert replay_resp.status_code == 200, replay_resp.text
        replay = EpisodeReplayResponse.model_validate(replay_resp.json())
        assert str(replay.id) == episode_id
        assert str(replay.user_session_id) == user_session_id
        assert replay.worker_session_id == worker_session_id
        assert replay.terminal_reason == TerminalReason.REJECTED_BY_REVIEW
        assert replay.failure_class == FailureClass.AGENT_QUALITY_FAILURE
        assert replay.detailed_status == EpisodeStatus.FAILED.value
        assert replay.entry_validation is not None
        assert replay.entry_validation.reason_code == "unsupported_validation_preview"
        assert replay.metadata_vars is not None
        assert (
            replay.metadata_vars.additional_info["entry_validation"]["reason_code"]
            == "unsupported_validation_preview"
        )
        assert replay.validation_result is not None
        assert Path(replay.validation_result.script_path) == Path("script.py")
        assert replay.validation_result.script_sha256
        assert replay.simulation_result is not None
        assert replay.simulation_result.fail_mode == "OUT_OF_BOUNDS"
        assert replay.review_manifests
        assert Path(replay.review_manifests[0].path) == Path(
            ".manifests/engineering_execution_handoff_manifest.json"
        )
        assert replay.review_manifests[0].manifest.reviewer_stage == (
            "engineering_execution_reviewer"
        )
        assert replay.review_decision_events
        assert replay.review_decision_events[0].decision == ReviewDecision.REJECTED
        assert replay.trace_ids.simulation_trace_ids
        assert replay.trace_ids.review_trace_ids
        assert replay.trace_ids.entry_validation_trace_ids
        assert replay.replay_artifacts
        artifact_hashes = {
            artifact.path: artifact.sha256 for artifact in replay.replay_artifacts
        }
        assert artifact_hashes["validation_results.json"]
        assert artifact_hashes["simulation_result.json"]
        assert artifact_hashes["script.py"]
        assert any(
            Path(artifact.path)
            == Path(".manifests/engineering_execution_handoff_manifest.json")
            for artifact in replay.replay_artifacts
        )
        assert any(
            Path(artifact.path).parent.name == "reviews"
            for artifact in replay.replay_artifacts
        )
        assert replay.assets
        assert any(
            Path(asset.s3_path) == Path("validation_results.json")
            for asset in replay.assets
        )
        assert any(
            Path(asset.s3_path) == Path("simulation_result.json")
            for asset in replay.assets
        )
        assert any(
            Path(asset.s3_path)
            == Path(".manifests/engineering_execution_handoff_manifest.json")
            for asset in replay.assets
        )

        replay_repeat_resp = await client.get(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/replay"
        )
        assert replay_repeat_resp.status_code == 200, replay_repeat_resp.text
        replay_repeat = EpisodeReplayResponse.model_validate(replay_repeat_resp.json())
        assert replay_repeat.worker_session_id == replay.worker_session_id
        assert replay_repeat.trace_ids == replay.trace_ids
        assert replay_repeat.replay_artifacts == replay.replay_artifacts
        assert replay_repeat.review_manifests == replay.review_manifests

        session_factory = get_sessionmaker()
        async with session_factory() as db:
            episode_row = await db.get(Episode, uuid.UUID(episode_id))
            assert episode_row is not None
            await _delete_workspace_file(
                client,
                session_id=worker_session_id,
                path="validation_results.json",
            )
            await db.execute(
                Trace.__table__.delete().where(
                    Trace.episode_id == uuid.UUID(episode_id)
                )
            )
            await db.execute(
                Asset.__table__.delete().where(
                    Asset.episode_id == uuid.UUID(episode_id)
                )
            )
            db.add(
                Asset(
                    episode_id=uuid.UUID(episode_id),
                    user_session_id=uuid.UUID(user_session_id),
                    asset_type=AssetType.PYTHON,
                    s3_path="script.py",
                    content="print('replay seed')\n",
                )
            )
            db.add(
                Asset(
                    episode_id=uuid.UUID(episode_id),
                    user_session_id=uuid.UUID(user_session_id),
                    asset_type=AssetType.OTHER,
                    s3_path="simulation_result.json",
                    content=replay.simulation_result.model_dump_json(indent=2),
                )
            )
            await db.commit()

        missing_resp = await client.get(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/replay"
        )
        assert missing_resp.status_code == 422, missing_resp.text
        assert "missing_artifact" in missing_resp.text

        (
            stale_episode_id,
            stale_worker_session_id,
            stale_user_session_id,
        ) = await _seed_replay_episode(
            client,
            manifest_revision="deadbeefdeadbeefdeadbeefdeadbeefdeadbeef",
        )
        stale_resp = await client.get(
            f"{CONTROLLER_URL}/api/episodes/{stale_episode_id}/replay"
        )
        assert stale_resp.status_code == 422, stale_resp.text
        assert "stale_manifest" in stale_resp.text
        assert stale_worker_session_id
        assert stale_user_session_id

        (
            unsupported_episode_id,
            unsupported_worker_session_id,
            unsupported_user_session_id,
        ) = await _seed_replay_episode(
            client,
            unsupported_mechanism="legacy-freefloating-actuator",
        )
        unsupported_resp = await client.get(
            f"{CONTROLLER_URL}/api/episodes/{unsupported_episode_id}/replay"
        )
        assert unsupported_resp.status_code == 422, unsupported_resp.text
        assert "unsupported_mechanism" in unsupported_resp.text
        assert unsupported_worker_session_id
        assert unsupported_user_session_id
