import io
import uuid

import pytest
from PIL import Image

from controller.agent.node_entry_validation import (
    validate_seeded_workspace_handoff_artifacts,
)
from controller.clients.worker import WorkerClient
from shared.enums import AgentName
from shared.workers.schema import (
    RenderArtifactMetadata,
    RenderManifest,
    RenderSiblingPaths,
)

WORKER_LIGHT_URL = "http://127.0.0.1:18001"


def _png_bytes(
    rgb: tuple[int, int, int],
    *,
    size: tuple[int, int] = (640, 480),
) -> bytes:
    image = Image.new("RGB", size, rgb)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def _render_manifest_json(evidence_path: str) -> str:
    manifest = RenderManifest(
        bundle_path="renders/final_solution_submission_renders",
        preview_evidence_paths=[evidence_path],
        artifacts={
            evidence_path: RenderArtifactMetadata(
                modality="unknown",
                group_key="render_evidence",
                siblings=RenderSiblingPaths(
                    rgb=evidence_path,
                ),
            )
        },
    )
    return manifest.model_dump_json(indent=2)


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_rejects_black_rgb_seed():
    session_id = f"INT-190-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        await worker.upload_file(
            "renders/images/render_e15_a0.png",
            _png_bytes((0, 0, 0)),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
    finally:
        await worker.aclose()

    render_errors = [error for error in errors if error.artifact_path == "renders"]
    assert render_errors, errors
    assert any("black/empty" in error.message for error in render_errors), render_errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_rejects_small_visible_rgb_seed_for_engineer_coder():
    session_id = f"INT-192-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        await worker.upload_file(
            "renders/images/render_e15_a0.png",
            _png_bytes((0, 255, 0), size=(2, 2)),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.ENGINEER_CODER,
        )
    finally:
        await worker.aclose()

    render_errors = [error for error in errors if error.artifact_path == "renders"]
    assert render_errors, errors
    assert any("too small" in error.message for error in render_errors), render_errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_allows_visible_rgb_seed():
    session_id = f"INT-191-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        await worker.upload_file(
            "renders/images/render_e15_a0.png",
            _png_bytes((0, 255, 0)),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
    finally:
        await worker.aclose()

    render_errors = [error for error in errors if error.artifact_path == "renders"]
    assert not render_errors, render_errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_rejects_missing_expected_render_image():
    session_id = f"INT-195-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    evidence_path = "renders/final_solution_submission_renders/render_evidence.png"
    try:
        await worker.upload_file(
            "renders/final_solution_submission_renders/render_manifest.json",
            _render_manifest_json(evidence_path).encode("utf-8"),
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.STEER,
        )
    finally:
        await worker.aclose()

    missing_image_errors = [
        error
        for error in errors
        if error.artifact_path == evidence_path and "render image" in error.message
    ]
    assert missing_image_errors, errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_rejects_missing_benchmark_render_evidence():
    session_id = f"INT-193-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.BENCHMARK_PLAN_REVIEWER,
        )
    finally:
        await worker.aclose()

    render_errors = [error for error in errors if error.artifact_path == "renders"]
    assert render_errors, errors
    assert any(
        "no render images" in error.message
        or "required render root missing" in error.message
        for error in render_errors
    ), render_errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_requires_motion_sidecars_for_video_evidence():
    session_id = f"INT-194-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    evidence_path = "renders/final_solution_submission_renders/render_evidence.mp4"
    try:
        await worker.upload_file(
            "renders/final_solution_submission_renders/render_manifest.json",
            _render_manifest_json(evidence_path).encode("utf-8"),
            bypass_agent_permissions=True,
        )
        await worker.upload_file(
            evidence_path,
            b"not-a-real-video-but-good-enough-for-contract-checking",
            bypass_agent_permissions=True,
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.STEER,
        )
    finally:
        await worker.aclose()

    missing_sidecar_paths = {
        error.artifact_path
        for error in errors
        if error.artifact_path
        in {
            "renders/final_solution_submission_renders/frames.jsonl",
            "renders/final_solution_submission_renders/objects.parquet",
        }
    }
    assert missing_sidecar_paths == {
        "renders/final_solution_submission_renders/frames.jsonl",
        "renders/final_solution_submission_renders/objects.parquet",
    }, errors
