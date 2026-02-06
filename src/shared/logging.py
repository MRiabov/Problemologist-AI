import logging
import os
import sys
from typing import Any

import structlog
from structlog.types import Processor

def configure_logging(service_name: str):
    """
    Configure structlog for the given service.
    
    In production (LOG_FORMAT=json), it outputs JSON.
    Otherwise, it outputs colored text for development.
    """
    log_format = os.getenv("LOG_FORMAT", "console").lower()
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    
    processors: list[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.processors.add_log_level,
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.CallsiteParameterAdder(
            {
                structlog.processors.CallsiteParameter.FILENAME,
                structlog.processors.CallsiteParameter.FUNC_NAME,
                structlog.processors.CallsiteParameter.LINENO,
            }
        ),
    ]

    # Add trace_id and span_id if they exist in contextvars
    # These are usually added via middleware or explicitly
    
    if log_format == "json":
        processors.append(structlog.processors.dict_tracebacks)
        processors.append(structlog.processors.JSONRenderer())
    else:
        processors.append(structlog.dev.ConsoleRenderer())

    structlog.configure(
        processors=processors,
        logger_factory=structlog.PrintLoggerFactory(),
        cache_logger_on_first_use=True,
    )

    # Root logger configuration for standard library logging
    handler = logging.StreamHandler(sys.stdout)
    if log_format == "json":
        # In production, we might want standard logs to be JSON too, 
        # but structlog usually handles that via its standard library integration if configured.
        pass

    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=getattr(logging, log_level, logging.INFO),
    )

def get_logger(name: str) -> structlog.BoundLogger:
    return structlog.get_logger(name)

def set_trace_context(trace_id: str, span_id: str | None = None):
    """Set trace information in the current context."""
    structlog.contextvars.bind_contextvars(trace_id=trace_id)
    if span_id:
        structlog.contextvars.bind_contextvars(span_id=span_id)
