import os
import pytest
from unittest.mock import patch

from controller.observability.langfuse import get_langfuse_callback, get_langfuse_client


def test_get_langfuse_callback_returns_none_without_credentials():
    with (
        patch.dict(os.environ, {}, clear=True),
        patch("controller.observability.langfuse.settings") as mock_settings,
    ):
        mock_settings.langfuse_public_key = None
        mock_settings.langfuse_secret_key = None
        mock_settings.is_integration_test = False
        callback = get_langfuse_callback(trace_id="trace-123", name="engineer_coder")
        assert callback is None


def test_get_langfuse_callback_uses_trace_context():
    env = {
        "LANGFUSE_PUBLIC_KEY": "pk-test",
        "LANGFUSE_SECRET_KEY": "sk-test",
        "LANGFUSE_HOST": "https://langfuse.example",
    }
    with (
        patch.dict(os.environ, env, clear=True),
        patch("controller.observability.langfuse.settings") as mock_settings,
        patch("controller.observability.langfuse.CallbackHandler") as mock_handler,
    ):
        mock_settings.langfuse_public_key = "pk-test"
        mock_settings.langfuse_secret_key = "sk-test"
        mock_settings.langfuse_host = "https://langfuse.example"
        mock_settings.is_integration_test = False
        get_langfuse_callback(trace_id="trace-123", name="engineer_coder")
        mock_handler.assert_called_once_with(
            public_key="pk-test",
            trace_context={
                "trace_id": "trace-123",
                "name": "engineer_coder",
                "tags": [],
                "metadata": {},
            },
        )


def test_get_langfuse_client_returns_none_without_credentials():
    with (
        patch.dict(os.environ, {}, clear=True),
        patch("controller.observability.langfuse.settings") as mock_settings,
    ):
        mock_settings.langfuse_public_key = None
        mock_settings.langfuse_secret_key = None
        mock_settings.is_integration_test = False
        client = get_langfuse_client()
        assert client is None


def test_get_langfuse_client_uses_configured_host():
    env = {
        "LANGFUSE_PUBLIC_KEY": "pk-test",
        "LANGFUSE_SECRET_KEY": "sk-test",
        "LANGFUSE_HOST": "https://langfuse.example",
    }
    with (
        patch.dict(os.environ, env, clear=True),
        patch("controller.observability.langfuse.settings") as mock_settings,
        patch("controller.observability.langfuse.Langfuse") as mock_langfuse,
    ):
        mock_settings.langfuse_public_key = "pk-test"
        mock_settings.langfuse_secret_key = "sk-test"
        mock_settings.langfuse_host = "https://langfuse.example"
        mock_settings.is_integration_test = False
        get_langfuse_client()
        mock_langfuse.assert_called_once_with(
            public_key="pk-test",
            secret_key="sk-test",
            host="https://langfuse.example",
        )


def test_get_langfuse_callback_returns_none_in_integration_test():
    with (
        patch("controller.observability.langfuse.settings") as mock_settings,
    ):
        mock_settings.langfuse_public_key = "pk-test"
        mock_settings.langfuse_secret_key = "sk-test"
        mock_settings.is_integration_test = True

        callback = get_langfuse_callback(trace_id="trace-123", name="test")
        assert callback is None

        client = get_langfuse_client()
        assert client is None


@pytest.mark.asyncio
async def test_calculate_and_report_automated_score():
    """Test that automated score is calculated from events and reported."""
    import uuid
    from unittest.mock import AsyncMock, MagicMock, patch

    from controller.observability.langfuse import calculate_and_report_automated_score
    from shared.enums import TraceType

    episode_id = uuid.uuid4()
    trace_id = "trace-123"
    agent_name = "cad_engineer"

    # Mock DB session
    mock_db = AsyncMock()
    mock_trace = MagicMock()
    # Use standard event format that map_events_to_prediction expects
    mock_trace.metadata_vars = {
        "event_type": "simulation_result",
        "success": True,
        "time_elapsed_s": 10.0,
        "compute_time_ms": 100.0,
    }
    mock_trace.trace_type = TraceType.EVENT

    mock_result = MagicMock()
    mock_result.scalars.return_value.all.return_value = [mock_trace]

    # Mock episode for metadata
    mock_episode = MagicMock()
    mock_episode.metadata_vars = {"max_unit_cost": 100.0, "max_weight_g": 1.0}

    # Set side effect for two calls to execute
    mock_db.execute.side_effect = [
        mock_result,  # First call for Trace
        MagicMock(scalar_one_or_none=lambda: mock_episode),  # Second call for Episode
    ]

    # Mock WorkerClient
    mock_worker = AsyncMock()
    mock_worker.read_file.return_value = (
        "constraints:\n  max_unit_cost: 100.0\n  max_weight_g: 1.0"
    )

    with patch("controller.observability.langfuse.report_score") as mock_report:
        await calculate_and_report_automated_score(
            episode_id=episode_id,
            session_id="session-123",
            trace_id=trace_id,
            agent_name=agent_name,
            db=mock_db,
            worker_client=mock_worker,
        )

        mock_report.assert_called_once()
        args, kwargs = mock_report.call_args
        assert kwargs["trace_id"] == trace_id
        assert kwargs["name"] == "automated_score"
        assert kwargs["value"] > 0
