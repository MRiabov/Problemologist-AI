import asyncio
import threading
from typing import Any

import structlog
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from controller.config.settings import settings

logger = structlog.get_logger(__name__)

# Async SQLAlchemy engines backed by asyncpg are not safe to share across event
# loops. Cache one engine/sessionmaker per loop instead.
_engine_cache: dict[int, Any] = {}
_sessionmaker_cache: dict[int, Any] = {}
_cache_lock = threading.RLock()


class Base(DeclarativeBase):
    pass


def _get_loop_id() -> int:
    try:
        return id(asyncio.get_running_loop())
    except RuntimeError:
        return 0


def get_engine():
    """Get or create an async engine for the current event loop."""
    global _engine_cache

    loop_id = _get_loop_id()
    with _cache_lock:
        if loop_id in _engine_cache:
            return _engine_cache[loop_id]

        database_url = settings.database_url
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

    loop_id = _get_loop_id()

    with _cache_lock:
        if loop_id in _sessionmaker_cache:
            return _sessionmaker_cache[loop_id]

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


async def dispose_all_engines() -> None:
    """Dispose cached async engines before the event loop shuts down."""
    with _cache_lock:
        engines = list(_engine_cache.values())
        _engine_cache.clear()
        _sessionmaker_cache.clear()

    for engine in engines:
        try:
            await engine.dispose()
        except Exception:
            logger.exception("engine_dispose_failed")
