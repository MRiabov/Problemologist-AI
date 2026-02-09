import pytest
import uuid
from unittest.mock import AsyncMock, MagicMock, patch
from fastapi import HTTPException
from controller.api.routes.episodes import report_trace_feedback, FeedbackRequest
from controller.persistence.models import Trace
from shared.enums import TraceType, ResponseStatus


@pytest.mark.asyncio
async def test_report_trace_feedback_success():
    episode_id = uuid.uuid4()
    trace_id = 1
    langfuse_trace_id = "langfuse-123"

    # Mock DB session
    db = AsyncMock()
    trace = Trace(
        id=trace_id,
        episode_id=episode_id,
        langfuse_trace_id=langfuse_trace_id,
        trace_type=TraceType.TOOL_START,
    )

    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = trace
    db.execute.return_value = mock_result

    # Mock Langfuse client
    mock_langfuse = MagicMock()

    with patch(
        "controller.api.routes.episodes.get_langfuse_client", return_value=mock_langfuse
    ):
        feedback = FeedbackRequest(score=1, comment="Good work!")
        response = await report_trace_feedback(episode_id, trace_id, feedback, db)

        assert response["status"] == ResponseStatus.ACCEPTED
        mock_langfuse.score.assert_called_once_with(
            trace_id=langfuse_trace_id,
            name="user-feedback",
            value=1,
            comment="Good work!",
        )

        # Verify local persistence
        assert trace.feedback_score == 1
        assert trace.feedback_comment == "Good work!"
        db.commit.assert_called_once()


@pytest.mark.asyncio
async def test_report_trace_feedback_no_langfuse_id():
    episode_id = uuid.uuid4()
    trace_id = 1

    db = AsyncMock()
    trace = Trace(
        id=trace_id,
        episode_id=episode_id,
        langfuse_trace_id=None,
        trace_type=TraceType.TOOL_START,
    )

    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = trace
    db.execute.return_value = mock_result

    with pytest.raises(HTTPException) as excinfo:
        await report_trace_feedback(episode_id, trace_id, FeedbackRequest(score=1), db)

    assert excinfo.value.status_code == 400
    assert "Trace does not have a Langfuse ID" in excinfo.value.detail
