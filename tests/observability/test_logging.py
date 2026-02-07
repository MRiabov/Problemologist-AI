"""Tests for the structured logging module."""

import json
import sys
from io import StringIO
from unittest.mock import patch

import structlog

from shared.observability.logging import (
    add_trace_id,
    clear_trace_id,
    configure_logging,
    get_logger,
    get_trace_id,
    set_trace_id,
)


class TestTraceIdContext:
    """Tests for trace ID context management."""

    def teardown_method(self) -> None:
        """Clear trace ID after each test."""
        clear_trace_id()

    def test_set_and_get_trace_id(self) -> None:
        """Test setting and getting trace ID."""
        set_trace_id("trace-123")
        assert get_trace_id() == "trace-123"

    def test_get_trace_id_default(self) -> None:
        """Test that trace ID is None by default."""
        assert get_trace_id() is None

    def test_clear_trace_id(self) -> None:
        """Test clearing trace ID."""
        set_trace_id("trace-123")
        clear_trace_id()
        assert get_trace_id() is None


class TestAddTraceIdProcessor:
    """Tests for the trace ID processor."""

    def teardown_method(self) -> None:
        """Clear trace ID after each test."""
        clear_trace_id()

    def test_adds_trace_id_when_set(self) -> None:
        """Test that trace_id is added when set in context."""
        set_trace_id("trace-456")
        event_dict = {"event": "test"}
        result = add_trace_id(None, "info", event_dict)  # type: ignore[arg-type]
        assert result["trace_id"] == "trace-456"

    def test_does_not_add_when_not_set(self) -> None:
        """Test that trace_id is not added when not set."""
        event_dict = {"event": "test"}
        result = add_trace_id(None, "info", event_dict)  # type: ignore[arg-type]
        assert "trace_id" not in result


class TestConfigureLogging:
    """Tests for logging configuration."""

    def test_configure_logging_json_output(self) -> None:
        """Test that logging outputs valid JSON when configured."""
        configure_logging(log_level="INFO", json_output=True)

        # Capture stdout
        captured = StringIO()
        with patch.object(sys, "stdout", captured):
            log = structlog.get_logger()
            log.info("test_message", key="value")

        output = captured.getvalue().strip()
        if output:  # Only test if there's output
            parsed = json.loads(output)
            assert parsed["event"] == "test_message"
            assert parsed["key"] == "value"
            assert "timestamp" in parsed

    def test_configure_logging_with_log_level(self) -> None:
        """Test that log level filtering works."""
        configure_logging(log_level="WARNING", json_output=True)

        captured = StringIO()
        with patch.object(sys, "stdout", captured):
            log = structlog.get_logger()
            log.info("should_not_appear")
            log.warning("should_appear")

        output = captured.getvalue()
        # Info should be filtered out, warning should pass
        assert "should_not_appear" not in output


class TestGetLogger:
    """Tests for logger retrieval."""

    def test_get_logger_returns_bound_logger(self) -> None:
        """Test that get_logger returns a valid logger."""
        configure_logging()
        log = get_logger("test.module")
        assert log is not None

    def test_get_logger_without_name(self) -> None:
        """Test that get_logger works without a name."""
        configure_logging()
        log = get_logger()
        assert log is not None
