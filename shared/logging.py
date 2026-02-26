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

    # Session context processor
    def add_session_context(logger, method_name, event_dict):
        # We can also add other context here
        return event_dict

    processors.insert(0, add_session_context)

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
        force=True,
    )

    # Ensure uvicorn loggers follow the global level and propagate to root
    for logger_name in ("uvicorn", "uvicorn.error", "uvicorn.access"):
        u_logger = logging.getLogger(logger_name)
        u_logger.handlers = []
        u_logger.propagate = True
        u_logger.setLevel(log_level_num)

    # Keep access logs at INFO for visibility during debugging
    access_logger = logging.getLogger("uvicorn.access")
    access_logger.setLevel(logging.INFO)


def get_logger(name: str) -> structlog.BoundLogger:
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
