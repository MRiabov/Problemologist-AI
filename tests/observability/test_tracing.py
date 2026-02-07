import os
from unittest.mock import patch

from shared.observability.tracing import get_trace_callback


def test_get_trace_callback_missing_env():
    """Test that it returns None when environment variables are missing."""
    with patch.dict(os.environ, {}, clear=True):
        handler = get_trace_callback()
        assert handler is None


def test_get_trace_callback_with_env():
    """Test that it initializes CallbackHandler with correct arguments from env."""
    env = {
        "LANGFUSE_PUBLIC_KEY": "pk-123",
        "LANGFUSE_SECRET_KEY": "sk-123",
        "LANGFUSE_HOST": "https://test.langfuse.com"
    }
    with patch.dict(os.environ, env):
        # Mock CallbackHandler to avoid actual initialization
        with patch("shared.observability.tracing.CallbackHandler") as mock_handler:
            get_trace_callback()
            mock_handler.assert_called_once_with(
                public_key="pk-123",
                secret_key="sk-123",
                host="https://test.langfuse.com"
            )


def test_get_trace_callback_default_host():
    """Test that it uses the default host if LANGFUSE_HOST is not set."""
    env = {
        "LANGFUSE_PUBLIC_KEY": "pk-123",
        "LANGFUSE_SECRET_KEY": "sk-123"
    }
    with patch.dict(os.environ, env, clear=True):
        with patch("shared.observability.tracing.CallbackHandler") as mock_handler:
            get_trace_callback()
            mock_handler.assert_called_once_with(
                public_key="pk-123",
                secret_key="sk-123",
                host="https://cloud.langfuse.com"
            )
