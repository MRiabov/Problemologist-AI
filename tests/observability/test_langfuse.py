import os
from unittest.mock import patch

from controller.observability.langfuse import get_langfuse_callback, get_langfuse_client


def test_get_langfuse_callback_returns_none_without_credentials():
    with patch.dict(os.environ, {}, clear=True):
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
    with patch.dict(os.environ, {}, clear=True):
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
        get_langfuse_client()
        mock_langfuse.assert_called_once_with(
            public_key="pk-test",
            secret_key="sk-test",
            host="https://langfuse.example",
        )
