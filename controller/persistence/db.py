import asyncio

from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase

from controller.config.settings import settings

# Cache engines per loop to avoid "Future attached to a different loop" errors
_engine_cache = {}


def get_engine():
    global _engine_cache

    # Get current loop (or create one if none exists, though typically one should exist)
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

    if loop in _engine_cache:
        return _engine_cache[loop]

    database_url = settings.postgres_url
    engine = create_async_engine(
        database_url,
        echo=False,
        pool_pre_ping=True,
        pool_size=20,
        max_overflow=10,
    )
    _engine_cache[loop] = engine
    return engine


def get_sessionmaker():
    return async_sessionmaker(
        bind=get_engine(),
        class_=AsyncSession,
        expire_on_commit=False,
    )


class Base(DeclarativeBase):
    pass


async def get_db():
    session_local = get_sessionmaker()
    async with session_local() as session:
        yield session
