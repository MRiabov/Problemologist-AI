import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from controller.persistence.models import GenerationSession

from controller.agent.benchmark.graph import run_generation_session
from controller.agent.benchmark.models import SessionStatus


@pytest.mark.asyncio
async def test_run_generation_session_exception_handling():
    """
    Verifies that run_generation_session correctly catches exceptions,
    updates the database status to 'failed', and logs the error.
    """
    session_id = uuid.uuid4()
    prompt = "Test prompt"

    # Mock initial state
    mock_session = MagicMock()
    mock_session.session_id = session_id
    mock_session.status = SessionStatus.planning

    initial_state = {
        "session": mock_session,
        "messages": [],
        "plan": {},
        "current_script": "",
        "simulation_result": None,
        "review_feedback": "",
        "iteration": 0,
    }

    # Mock define_graph to return an app whose astream raises an exception
    mock_app = MagicMock()

    async def mock_astream(*args, **kwargs):
        # We simulate some output then an exception
        yield {"planner": {"plan": "some plan"}}
        raise RuntimeError("LLM Failure: Out of credits")

    mock_app.astream = mock_astream

    # Mock database session and update statement
    mock_db = AsyncMock()
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.return_value = mock_db

    # Mock Episode object
    mock_episode = MagicMock()
    mock_episode.metadata_vars = {}
    mock_episode.status = "running"

    # Mock db.get for the exception handler
    mock_db.get = AsyncMock(return_value=mock_episode)

    async def mock_execute_graph_streaming(*args, **kwargs):
        raise RuntimeError("LLM Failure: Out of credits")

    with (
        patch("controller.agent.benchmark.graph.define_graph", return_value=mock_app),
        patch(
            "controller.agent.benchmark.graph.get_sessionmaker",
            return_value=mock_session_factory,
        ),
        patch(
            "controller.agent.benchmark.graph._execute_graph_streaming",
            side_effect=mock_execute_graph_streaming,
        ),
    ):
        # Execute the session
        final_state = await run_generation_session(prompt, session_id=session_id)

        # Verify status update to failed happened in state
        assert final_state["session"].status == SessionStatus.failed

        # Check that DB was updated
        mock_db.get.assert_called()
        assert mock_episode.status == "failed"
        assert mock_episode.metadata_vars["detailed_status"] == SessionStatus.failed
        assert "LLM Failure: Out of credits" in mock_episode.metadata_vars["error"]
        mock_db.commit.assert_called()
        # Execute the session
        final_state = await run_generation_session(prompt, session_id=session_id)

        # Verify status update to failed happened in state
        assert final_state["session"].status == SessionStatus.failed

        # Check that DB was updated
        mock_db.get.assert_called()
        assert mock_episode.status == "failed"
        assert mock_episode.metadata_vars["detailed_status"] == SessionStatus.failed
        assert "LLM Failure: Out of credits" in mock_episode.metadata_vars["error"]
        mock_db.commit.assert_called()
