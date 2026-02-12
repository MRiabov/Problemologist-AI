import logging
import os
import sys

import structlog
from structlog.types import Processor


def configure_logging(_service_name: str):
    """
    Configure structlog for the given service.

    In production (LOG_FORMAT=json), it outputs JSON.
    Otherwise, it outputs colored text for development.
    """
    log_format = os.getenv("LOG_FORMAT", "console").lower()
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    log_level_num = getattr(logging, log_level, logging.INFO)

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
        wrapper_class=structlog.make_filtering_bound_logger(log_level_num),
        cache_logger_on_first_use=True,
    )

    # Root logger configuration for standard library logging
    if log_format == "json":
        # In production, we might want standard logs to be JSON too
        pass

    logging.basicConfig(
        format="%(message)s",
        stream=sys.stdout,
        level=log_level_num,
    )

    # Ensure uvicorn loggers follow the global level
    logging.getLogger("uvicorn").setLevel(log_level_num)
    logging.getLogger("uvicorn.error").setLevel(log_level_num)

    # Demote uvicorn access logs to DEBUG so they don't clutter INFO/WARNING
    access_logger = logging.getLogger("uvicorn.access")
    access_logger.setLevel(logging.DEBUG)

    class AccessLogFilter(logging.Filter):
        def filter(self, record):
            if record.levelno == logging.INFO:
                record.levelno = logging.DEBUG
                record.levelname = "DEBUG"
            return True

    access_logger.addFilter(AccessLogFilter())


def get_logger(name: str) -> structlog.BoundLogger:
    return structlog.get_logger(name)


def set_trace_context(trace_id: str, span_id: str | None = None):
    """Set trace information in the current context."""
    structlog.contextvars.bind_contextvars(trace_id=trace_id)
    if span_id:
        structlog.contextvars.bind_contextvars(span_id=span_id)
