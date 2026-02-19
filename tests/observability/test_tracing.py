import os
from unittest.mock import patch

from shared.observability.tracing import get_trace_callback


def test_get_trace_callback_missing_env():
    """Test that it returns None when environment variables are missing."""
    with patch.dict(os.environ, {}, clear=True):
        handler = get_trace_callback()
        assert handler is None


def test_get_trace_callback_with_env():
    """Test that it initializes CallbackHandler if environment variables are set."""
    env = {
        "LANGFUSE_PUBLIC_KEY": "dummy_public_key",
        "LANGFUSE_SECRET_KEY": "dummy_secret_key",
        "LANGFUSE_HOST": "https://test.langfuse.com",
    }
    with (
        patch.dict(os.environ, env),
        patch("shared.observability.tracing.CallbackHandler") as mock_handler,
    ):
        get_trace_callback()
        mock_handler.assert_called_once_with()


def test_get_trace_callback_default_host():
    """Test that it works even if LANGFUSE_HOST is not set."""
    env = {"LANGFUSE_PUBLIC_KEY": "dummy_public_key", "LANGFUSE_SECRET_KEY": "dummy_secret_key"}
    with (
        patch.dict(os.environ, env, clear=True),
        patch("shared.observability.tracing.CallbackHandler") as mock_handler,
    ):
        get_trace_callback()
        mock_handler.assert_called_once_with()
