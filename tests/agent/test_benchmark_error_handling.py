import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from controller.agent.benchmark.schema import GenerationSessionModel

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
    mock_session_factory = MagicMock(return_value=mock_db)

    with (
        patch("controller.agent.benchmark.graph.define_graph", return_value=mock_app),
        patch(
            "controller.agent.benchmark.graph.get_sessionmaker",
            return_value=mock_session_factory,
        ),
        patch("controller.agent.benchmark.graph.update") as mock_update,
    ):
        mock_update_obj = MagicMock()
        mock_update.return_value = mock_update_obj
        mock_update_obj.where.return_value = mock_update_obj
        mock_update_obj.values.return_value = mock_update_obj

        # Execute the session
        final_state = await run_generation_session(prompt, session_id=session_id)

        # Verify status update to failed happened in DB
        assert final_state["session"].status == SessionStatus.failed

        # Check that update was called with status=failed
        # The second call in the loop (Exception handler) should set it to failed
        mock_update_obj.values.assert_any_call(
            status=SessionStatus.failed,
            validation_logs=GenerationSessionModel.validation_logs
            + ["Error: LLM Failure: Out of credits"],
        )
