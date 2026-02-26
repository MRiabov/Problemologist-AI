import asyncio
import threading
from typing import Any

import structlog
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from controller.config.settings import settings

logger = structlog.get_logger(__name__)

# Global engine used across all event loops (engines are thread-safe and manage their own pools)
_global_engine: Any = None
_sessionmaker_cache: dict[int, Any] = {}
_cache_lock = threading.RLock()


class Base(DeclarativeBase):
    pass


def get_engine():
    """Get or create the global async engine."""
    global _global_engine

    with _cache_lock:
        if _global_engine is not None:
            return _global_engine

        database_url = settings.database_url
        _global_engine = create_async_engine(
            database_url,
            echo=False,
            pool_pre_ping=True,
            pool_size=20,
            max_overflow=10,
        )
        return _global_engine


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
