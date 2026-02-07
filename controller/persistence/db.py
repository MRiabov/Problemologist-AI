import os
from functools import lru_cache

from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase


# Default to sqlite for local dev if POSTGRES_URL is not provided,
# but requirement says POSTGRES_URL.
# SQLAlchemy async postgres uses postgresql+asyncpg://
@lru_cache
def get_engine():
    DATABASE_URL = os.getenv(
        "POSTGRES_URL",
        "postgresql+asyncpg://postgres:postgres@localhost:5432/problemologist",
    )
    return create_async_engine(DATABASE_URL, echo=False)


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
        try:
            yield session
        finally:
            await session.close()
