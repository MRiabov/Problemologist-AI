import io
import uuid

import pytest
from PIL import Image

from controller.agent.node_entry_validation import (
    validate_seeded_workspace_handoff_artifacts,
)
from controller.clients.worker import WorkerClient
from shared.enums import AgentName

WORKER_LIGHT_URL = "http://127.0.0.1:18001"


def _png_bytes(rgb: tuple[int, int, int]) -> bytes:
    image = Image.new("RGB", (2, 2), rgb)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


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
async def test_render_validation_rejects_black_rgb_seed_for_engineer_coder():
    session_id = f"INT-192-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        await worker.upload_file(
            "renders/images/render_e15_a0.png",
            _png_bytes((0, 0, 0)),
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
    assert any("black/empty" in error.message for error in render_errors), render_errors


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_render_validation_allows_visible_rgb_seed():
    session_id = f"INT-191-{uuid.uuid4().hex[:8]}"
    worker = WorkerClient(base_url=WORKER_LIGHT_URL, session_id=session_id)
    try:
        await worker.upload_file(
            "renders/images/render_e15_a0.png",
            _png_bytes((0, 255, 0)),
        )

        errors = await validate_seeded_workspace_handoff_artifacts(
            worker_client=worker,
            target_node=AgentName.ENGINEER_EXECUTION_REVIEWER,
        )
    finally:
        await worker.aclose()

    render_errors = [error for error in errors if error.artifact_path == "renders"]
    assert not render_errors, render_errors
