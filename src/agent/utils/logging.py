import logging
import sys
from typing import Any

import structlog


def setup_logging(level: str = "INFO"):
    """Configures structured logging for the project."""

    # Common processors for both console and JSON logging
    processors = [
        structlog.contextvars.merge_contextvars,
        structlog.processors.add_log_level,
        structlog.processors.StackInfoRenderer(),
        structlog.dev.set_exc_info,
        structlog.processors.TimeStamper(fmt="iso"),
    ]

    # Console-friendly output for interactive use
    if sys.stderr.isatty():
        processors.append(structlog.dev.ConsoleRenderer())
    else:
        # JSON output for logs (e.g., in production or CI)
        processors.append(structlog.processors.dict_tracebacks)
        processors.append(structlog.processors.JSONRenderer())

    structlog.configure(
        processors=processors,
        logger_factory=structlog.PrintLoggerFactory(),
        wrapper_class=structlog.make_filtering_bound_logger(
            getattr(logging, level.upper())
        ),
        cache_logger_on_first_use=True,
    )


def get_logger(name: str) -> Any:
    """Returns a bound logger for a specific component."""
    return structlog.get_logger(name)
