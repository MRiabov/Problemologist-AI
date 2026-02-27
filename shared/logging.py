import logging
import os
import sys

import structlog
from structlog.types import Processor


def configure_logging(service_name: str):
    """
    Configure structlog for the given service.

    In production (LOG_FORMAT=json), it outputs JSON.
    Otherwise, it outputs colored text for development.

    Supports EXTRA_DEBUG_LOG environment variable to output DEBUG logs to a file
    while keeping stdout at LOG_LEVEL.
    """
    log_format = os.getenv("LOG_FORMAT", "console").lower()
    log_level = os.getenv("LOG_LEVEL", "INFO").upper()
    log_level_num = getattr(logging, log_level, logging.INFO)

    # Extra debug log file
    extra_debug_log = os.getenv("EXTRA_DEBUG_LOG")

    processors: list[Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.stdlib.add_log_level,
        structlog.stdlib.add_logger_name,
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

    # Session context processor
    def add_service_context(logger, method_name, event_dict):
        event_dict["service"] = service_name
        return event_dict

    processors.insert(0, add_service_context)

    if log_format == "json":
        processors.append(structlog.processors.dict_tracebacks)
        processors.append(structlog.processors.JSONRenderer())
    else:
        processors.append(structlog.dev.ConsoleRenderer())

    structlog.configure(
        processors=processors,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )

    # Root logger configuration
    root_logger = logging.getLogger()

    # Clear existing handlers to avoid duplicates
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level_num)
    console_handler.setFormatter(logging.Formatter("%(message)s"))
    root_logger.addHandler(console_handler)

    # Optional file handler for DEBUG logs
    if extra_debug_log:
        # Ensure directory exists
        extra_debug_log_abs = os.path.abspath(extra_debug_log)
        os.makedirs(os.path.dirname(extra_debug_log_abs), exist_ok=True)
        file_handler = logging.FileHandler(extra_debug_log_abs)
        file_handler.setLevel(logging.DEBUG)

        # We use a simple formatter because structlog already formatted the message
        file_handler.setFormatter(logging.Formatter("%(message)s"))
        root_logger.addHandler(file_handler)

        # If we have a debug file, root must be at least DEBUG to let messages through
        root_logger.setLevel(logging.DEBUG)
    else:
        root_logger.setLevel(log_level_num)

    # Ensure uvicorn loggers follow the global level and propagate to root
    for logger_name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
        u_logger = logging.getLogger(logger_name)
        u_logger.handlers = []
        u_logger.propagate = True
        u_logger.setLevel(logging.DEBUG if extra_debug_log else log_level_num)

    # Keep access logs at INFO for visibility during debugging unless debug file is on
    if not extra_debug_log:
        access_logger = logging.getLogger("uvicorn.access")
        access_logger.setLevel(logging.INFO)


def get_logger(name: str) -> structlog.stdlib.BoundLogger:
    return structlog.get_logger(name)


def set_trace_context(trace_id: str, span_id: str | None = None):
    """Set trace information in the current context."""
    structlog.contextvars.bind_contextvars(trace_id=trace_id)
    if span_id:
        structlog.contextvars.bind_contextvars(span_id=span_id)


def log_marker_middleware():
    """
    Returns a FastAPI middleware that extracts session and test IDs
    and binds them to the structlog context.
    """
    from fastapi import Request
    from starlette.middleware.base import BaseHTTPMiddleware

    class LogMarkerMiddleware(BaseHTTPMiddleware):
        async def dispatch(self, request: Request, call_next):
            # Extract markers
            session_id = request.headers.get("X-Session-ID")
            test_id = request.headers.get("X-Integration-Test-ID")
            query_marker = request.query_params.get("marker")

            # Clear context and bind new markers
            structlog.contextvars.clear_contextvars()
            if session_id:
                structlog.contextvars.bind_contextvars(session_id=session_id)
            if test_id:
                structlog.contextvars.bind_contextvars(test_id=test_id)

            if query_marker:
                # Log a prominent marker if requested via query param
                logger = structlog.get_logger("marker")
                logger.info(f"\n\n{'=' * 20} {query_marker} {'=' * 20}\n")

            response = await call_next(request)
            return response

    return LogMarkerMiddleware
