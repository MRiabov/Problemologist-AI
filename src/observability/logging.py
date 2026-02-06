"""
Structured logging configuration using structlog.

This module configures structlog to emit JSON-formatted logs with
ISO timestamps and optional trace correlation IDs.
"""

import logging
import sys
from contextvars import ContextVar
from typing import Any

import structlog
from structlog.types import Processor

# Context variable to store trace_id for correlation
_trace_id_ctx: ContextVar[str | None] = ContextVar("trace_id", default=None)


def set_trace_id(trace_id: str) -> None:
    """
    Set the current trace ID for log correlation.

    Args:
        trace_id: The trace identifier to associate with subsequent logs.
    """
    _trace_id_ctx.set(trace_id)


def get_trace_id() -> str | None:
    """
    Get the current trace ID from context.

    Returns:
        The current trace ID or None if not set.
    """
    return _trace_id_ctx.get()


def clear_trace_id() -> None:
    """Clear the current trace ID from context."""
    _trace_id_ctx.set(None)


def add_trace_id(
    _logger: logging.Logger, _method_name: str, event_dict: dict[str, Any]
) -> dict[str, Any]:
    """
    Structlog processor that adds trace_id to log events if available.

    Args:
        logger: The wrapped logger object.
        method_name: The name of the wrapped method.
        event_dict: The event dictionary to process.

    Returns:
        The event dictionary with trace_id added if available.
    """
    trace_id = _trace_id_ctx.get()
    if trace_id:
        event_dict["trace_id"] = trace_id
    return event_dict


def configure_logging(
    *,
    log_level: str = "INFO",
    json_output: bool = True,
    add_timestamp: bool = True,
) -> None:
    """
    Configure structlog for structured JSON logging.

    This function sets up structlog with:
    - ISO-formatted timestamps
    - JSON renderer for machine-readable output
    - Trace ID correlation (if set in context)
    - Standard library logging integration

    Args:
        log_level: The minimum log level (DEBUG, INFO, WARNING, ERROR, CRITICAL).
        json_output: If True, output logs as JSON. If False, output as plain text.
        add_timestamp: If True, add ISO-formatted timestamps to logs.

    Example:
        >>> configure_logging(log_level="DEBUG")
        >>> log = structlog.get_logger()
        >>> log.info("Application started", version="1.0.0")
    """
    # Build the processor chain
    processors: list[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        add_trace_id,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
    ]

    if add_timestamp:
        processors.insert(0, structlog.processors.TimeStamper(fmt="iso"))

    if json_output:
        processors.append(structlog.processors.JSONRenderer())
    else:
        processors.append(structlog.dev.ConsoleRenderer())

    # Configure structlog
    structlog.configure(
        processors=processors,
        wrapper_class=structlog.stdlib.BoundLogger,
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        cache_logger_on_first_use=True,
    )

    # Configure standard library logging to use structlog
    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=getattr(logging, log_level.upper()),
    )


def get_logger(name: str | None = None) -> structlog.stdlib.BoundLogger:
    """
    Get a structured logger instance.

    Args:
        name: Optional logger name. If None, uses the calling module's name.

    Returns:
        A configured structlog BoundLogger instance.

    Example:
        >>> log = get_logger(__name__)
        >>> log.info("Processing request", request_id="abc123")
    """
    return structlog.get_logger(name)
