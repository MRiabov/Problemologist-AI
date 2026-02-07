import contextlib
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from worker.generators.benchmark.graph import run_generation_session
from worker.generators.benchmark.models import SessionStatus


@pytest.mark.asyncio
async def test_benchmark_generation_e2e():
    # Mock DB Session
    mock_db = AsyncMock()
    mock_db.commit = AsyncMock()
    mock_db.execute = AsyncMock()
    mock_db.add = MagicMock()
    mock_db.refresh = AsyncMock()

    # Mock get_db async context manager
    @contextlib.asynccontextmanager
    async def mock_get_db_gen():
        yield mock_db

    # Mock S3
    mock_boto = MagicMock()
    mock_s3_client = MagicMock()
    mock_boto.client.return_value = mock_s3_client

    # Mock Graph App
    mock_app = MagicMock()

    # Define a mock stream that simulates a successful run
    async def mock_astream(initial_state):
        # Planner
        yield {"planner": {**initial_state, "plan": {"theme": "stack"}}}
        # Coder
        state_coder = {
            **initial_state,
            "current_script": "print('hello')",
            "plan": {"theme": "stack"},
        }
        yield {"coder": state_coder}
        # Validator
        state_val = {
            **state_coder,
            "mjcf_content": "<mujoco/>",
            "simulation_result": {
                "valid": True,
                "render_paths": ["/tmp/img1.png", "/tmp/img2.png"],
            },
        }
        yield {"validator": state_val}
        # Reviewer
        state_rev = {**state_val, "review_feedback": "Approved"}
        yield {"reviewer": state_rev}

    mock_app.astream = mock_astream

    # Patch dependencies
    # Note: We patch where they are IMPORTED in graph.py and storage.py
    with (
        patch("worker.generators.benchmark.graph.get_db", side_effect=mock_get_db_gen),
        patch("worker.generators.benchmark.storage.boto3", mock_boto),
        patch("worker.generators.benchmark.graph.define_graph", return_value=mock_app),
        patch(
            "worker.generators.benchmark.graph.Path.read_bytes",
            return_value=b"fake_bytes",
        ),
    ):
        prompt = "Create a stack of blocks"
        final_state = await run_generation_session(prompt)

        # Assertions

        # 1. Final Status
        assert final_state["session"].status == SessionStatus.accepted

        # 2. DB Interactions
        # Should add session and asset (2 calls)
        assert mock_db.add.call_count == 2
        # Should update status multiple times (at least planner, coder, reviewer)
        assert mock_db.execute.call_count >= 3

        # 3. S3 Interactions
        # script, mjcf upload via put_object
        assert mock_s3_client.put_object.call_count >= 2
        # zip upload via upload_fileobj
        assert mock_s3_client.upload_fileobj.call_count >= 1

    print("E2E Test Passed!")
