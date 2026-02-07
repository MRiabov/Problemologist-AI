import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from langgraph.checkpoint.postgres import PostgresSaver
from psycopg_pool import AsyncConnectionPool


def get_db_url() -> str:
    """
    Returns the database URL from the DATABASE_URL environment variable.
    
    Expected format: postgresql://user:password@host:port/dbname
    """
    url = os.getenv("DATABASE_URL")
    if not url:
        # We raise an error if the database URL is missing as persistence is critical.
        raise ValueError("DATABASE_URL environment variable is not set")
    
    # Ensure the URL is in a format psycopg can understand (no asyncpg+ prefix)
    if "postgresql+asyncpg://" in url:
        url = url.replace("postgresql+asyncpg://", "postgresql://")
    elif "postgresql+psycopg://" in url:
        url = url.replace("postgresql+psycopg://", "postgresql://")
        
    return url


class ControllerCheckpointSaver:
    """
    Persistence layer for LangGraph using Postgres.
    Utilizes langgraph.checkpoint.postgres.PostgresSaver.
    """
    
    @staticmethod
    @asynccontextmanager
    async def create_saver() -> AsyncIterator[PostgresSaver]:
        """
        Context manager that yields a configured PostgresSaver.
        Ensures the connection pool is properly managed.
        """
        conn_string = get_db_url()
        async with AsyncConnectionPool(conn_string, max_size=20) as pool:
            saver = PostgresSaver(pool)
            yield saver


async def setup_persistence():
    """
    Initializes the Postgres tables required for state persistence.
    Should be called during application startup.
    """
    async with ControllerCheckpointSaver.create_saver() as saver:
        await saver.setup()
