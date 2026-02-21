import asyncio
import os
import uuid

import httpx
import pytest
from temporalio.client import Client


# Local constants for black-box testing
class EpisodeStatus:
    COMPLETED = "completed"
    FAILED = "failed"
    RUNNING = "running"


class AssetType:
    VIDEO = "video"
    RENDER = "render"
    MJCF = "mjcf"


CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
TEMPORAL_URL = os.getenv("TEMPORAL_URL", "localhost:17233")


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_053_temporal_workflow_lifecycle():
    """INT-053: Verify Temporal workflow lifecycle persistence."""
    async with httpx.AsyncClient() as client:
        # 1. Create a dummy episode to link to
        task = "Test Temporal Workflow Lifecycle"
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": task, "session_id": "INT-053-obs"},
        )
        assert resp.status_code == 202
        episode_id = resp.json()["episode_id"]

        # 2. Connect to Temporal and start SimulationWorkflow
        temporal = await Client.connect(TEMPORAL_URL)

        # We use a dummy compound_json that will be "compiled" by mock activity
        params = {"compound_json": "{}", "episode_id": str(episode_id)}

        handle = await temporal.start_workflow(
            "SimulationWorkflow",
            params,
            id=f"test-sim-{episode_id}",
            task_queue="simulation-task-queue",
        )

        # 3. Wait for workflow completion
        await handle.result()

        # 4. Verify episode status in DB via API
        status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        assert status_resp.status_code == 200
        data = status_resp.json()
        assert data["status"] == EpisodeStatus.COMPLETED
        assert data["updated_at"] is not None


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_055_s3_artifact_upload_logging():
    """INT-055: Verify S3 artifact upload logging and linkage."""
    async with httpx.AsyncClient() as client:
        # 1. Create episode
        episode_id = str(uuid.uuid4())
        # Manual insert or use agent/run
        await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": "Test S3 Upload", "session_id": "INT-055-s3"},
        )
        # Wait, I need the actual ID. Let's list and pick latest?
        # Or better just use the one from previous test if I want to be fast, but isolation is better.

        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": "Test S3 Upload", "session_id": "INT-055-s3"},
        )
        episode_id = resp.json()["episode_id"]

        # 2. Trigger workflow
        temporal = await Client.connect(TEMPORAL_URL)
        await temporal.execute_workflow(
            "SimulationWorkflow",
            {"compound_json": "{}", "episode_id": str(episode_id)},
            id=f"test-s3-{episode_id}",
            task_queue="simulation-task-queue",
        )

        # 3. Verify Asset record via API
        episode_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        data = episode_resp.json()

        assert len(data["assets"]) > 0
        asset = data["assets"][0]
        assert asset["asset_type"] == AssetType.VIDEO
        assert asset["s3_path"].startswith("videos/")
        assert asset["created_at"] is not None


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_054_temporal_failure_path():
    """INT-054: Verify Temporal outage/failure logging path (via failure injection)."""
    async with httpx.AsyncClient() as client:
        # 1. Create episode via API
        resp = await client.post(
            f"{CONTROLLER_URL}/test/episodes",
            json={"task": "Test Failure Injection", "session_id": "INT-054-fail"},
        )
        assert resp.status_code == 201
        episode_id = resp.json()["episode_id"]

        # 2. Trigger workflow with MJCF failure injection
        temporal = await Client.connect(TEMPORAL_URL)

        params = {
            "compound_json": "{}",
            "episode_id": str(episode_id),
            "simulate_failures": {"mjcf_compilation": True},
        }

        handle = await temporal.start_workflow(
            "SimulationWorkflow",
            params,
            id=f"test-fail-{episode_id}",
            task_queue="simulation-task-queue",
        )

        # 3. Wait for workflow failure (it should fail after 3 attempts due to our retry policy)
        with pytest.raises(Exception):
            await handle.result()

        # 4. Verify episode status is FAILED in DB
        status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        assert status_resp.status_code == 200
        data = status_resp.json()
        assert data["status"] == EpisodeStatus.FAILED


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_056_s3_upload_failure_retry():
    """INT-056: Verify S3 upload failure + retry logging."""
    # Similar issue with retries.
    # But here we WANT to verify retry logic.
    # Temporal retries by default.
    # We can verify that the episode does NOT go to completed status quickly.

    async with httpx.AsyncClient() as client:
        # Create episode via test endpoint (no agent task interference)
        resp = await client.post(
            f"{CONTROLLER_URL}/test/episodes",
            json={"task": "Test S3 Retry", "session_id": "INT-056-retry"},
        )
        episode_id = resp.json()["episode_id"]

        temporal = await Client.connect(TEMPORAL_URL)
        handle = await temporal.start_workflow(
            "SimulationWorkflow",
            {
                "compound_json": "{}",
                "episode_id": episode_id,
                "simulate_failures": {"s3_upload": True},
            },
            id=f"test-retry-{episode_id}",
            task_queue="simulation-task-queue",
        )

        # Wait a bit. It should be retrying.
        # With maximum_attempts=3 and 1s initial backoff, it fails around 3-4s.
        # So we check at 1s.
        await asyncio.sleep(1)

        status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        data = status_resp.json()
        assert data["status"] == EpisodeStatus.RUNNING  # Should still be running (retrying)

        # Cleanup: Cancel it so we don't spam logs
        await handle.cancel()
