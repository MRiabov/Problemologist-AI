import asyncio
import threading
from typing import Any

import structlog
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from controller.config.settings import settings

logger = structlog.get_logger(__name__)

# Cache engines and sessionmakers per event loop to avoid cross-loop usage errors
# in highly concurrent environments like integration tests.
_engine_cache: dict[int, Any] = {}
_sessionmaker_cache: dict[int, Any] = {}
_cache_lock = threading.RLock()


class Base(DeclarativeBase):
    pass


def get_engine():
    """Get or create a cached async engine for the current event loop."""
    global _engine_cache

    try:
        loop = asyncio.get_running_loop()
        loop_id = id(loop)
    except RuntimeError:
        # Fallback for non-async contexts or setup
        loop_id = 0

    with _cache_lock:
        if loop_id in _engine_cache:
            return _engine_cache[loop_id]

        database_url = settings.database_url
        logger.info("creating_new_engine", loop_id=loop_id, url=database_url)
        engine = create_async_engine(
            database_url,
            echo=False,
            pool_pre_ping=True,
            pool_size=20,
            max_overflow=10,
        )
        _engine_cache[loop_id] = engine
        return engine


def get_sessionmaker():
    """Get or create a cached sessionmaker for the current event loop."""
    global _sessionmaker_cache

    try:
        loop = asyncio.get_running_loop()
        loop_id = id(loop)
    except RuntimeError:
        loop_id = 0

    with _cache_lock:
        if loop_id in _sessionmaker_cache:
            return _sessionmaker_cache[loop_id]

        logger.info("creating_new_sessionmaker", loop_id=loop_id)
        engine = get_engine()
        sm = async_sessionmaker(
            engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )
        _sessionmaker_cache[loop_id] = sm
        return sm


async def get_db():
    """Dependency for FastAPI to provide an async DB session."""
    session_local = get_sessionmaker()
    async with session_local() as session:
        yield session
