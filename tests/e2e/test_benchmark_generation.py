import contextlib
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.agent.benchmark.graph import run_generation_session
from controller.agent.benchmark.models import SessionStatus


@pytest.mark.asyncio
async def test_benchmark_generation_e2e():
    # Mock DB Session
    mock_db = AsyncMock()
    mock_db.commit = AsyncMock()
    mock_db.execute = AsyncMock()
    mock_db.add = MagicMock()
    mock_db.refresh = AsyncMock()
    mock_db.get = AsyncMock()

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
    async def mock_astream(initial_state, **kwargs):
        # Planner
        from shared.simulation.schemas import RandomizationStrategy

        mock_plan = RandomizationStrategy(theme="stack", reasoning="test")
        yield {"planner": {"plan": mock_plan}}
        # Coder
        yield {"coder": {"current_script": "print('hello')"}}
        # Validator
        yield {
            "validator": {
                "simulation_result": MagicMock(
                    valid=True, render_paths=["/tmp/img1.png"], render_data=[b"fake"]
                )
            }
        }
        # Reviewer
        yield {"reviewer": {"review_feedback": "Approved"}}

    mock_app.astream = mock_astream

    # Patch dependencies
    # Note: We patch where they are IMPORTED in graph.py and storage.py
    with (
        patch(
            "controller.agent.benchmark.graph.get_sessionmaker",
            return_value=mock_get_db_gen,
        ),
        patch("controller.agent.benchmark.storage.boto3", mock_boto),
        patch("controller.agent.benchmark.graph.define_graph", return_value=mock_app),
    ):
        prompt = "Create a stack of blocks"
        final_state = await run_generation_session(prompt)

        # Assertions

        # 1. Final Status
        # The graph now pauses after planning for user confirmation
        assert final_state.session.status == SessionStatus.planned

        # 2. DB Interactions
        # Should add episode (1 call)
        assert mock_db.add.call_count >= 1
        # Should update status (commit)
        assert mock_db.commit.call_count >= 1

    print("E2E Test Passed!")
