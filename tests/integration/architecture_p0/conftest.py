import pytest
import pytest_asyncio
import os
import httpx
from unittest.mock import AsyncMock, patch, MagicMock

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://localhost:18002")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest_asyncio.fixture
async def controller_client():
    # Try connecting to real service
    try:
        async with httpx.AsyncClient(base_url=CONTROLLER_URL, timeout=0.5) as client:
            resp = await client.get("/health")
            if resp.status_code == 200:
                yield client
                return
    except Exception:
        pass

    # If not reachable, run in-process
    os.environ["DATABASE_URL"] = "dummy"
    os.environ["BACKUP_S3_BUCKET"] = "dummy"
    os.environ["ASSET_S3_BUCKET"] = "dummy"

    # Use in-memory SQLite for tests
    from controller.config.settings import settings

    settings.postgres_url = "sqlite+aiosqlite:///:memory:"

    # Ensure models are imported so tables are created
    import controller.persistence.models

    # Patch create_async_engine to strip invalid args for SQLite
    from controller.persistence import db
    from sqlalchemy.pool import StaticPool

    original_create_async_engine = db.create_async_engine

    def mocked_create_async_engine(url, **kwargs):
        if "sqlite" in url:
            kwargs.pop("pool_size", None)
            kwargs.pop("max_overflow", None)
            kwargs["poolclass"] = StaticPool
            kwargs["connect_args"] = {"check_same_thread": False}
        return original_create_async_engine(url, **kwargs)

    with patch(
        "controller.persistence.db.create_async_engine",
        side_effect=mocked_create_async_engine,
    ):
        # Create tables
        # Need to clear engine cache first
        db._engine_cache.clear()

        from controller.persistence.db import get_engine, Base

        # Initialize engine and create tables
        engine = get_engine()
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

        # Ensure the app uses THIS engine instance
        def mocked_get_engine_singleton():
            return engine

        with patch(
            "controller.persistence.db.get_engine",
            side_effect=mocked_get_engine_singleton,
        ):
            # Patch lifespan to avoid real startup logic (Temporal connection)
            with patch("controller.api.main.lifespan", side_effect=AsyncMock()):
                from controller.api.main import app

                # Mock temporal client in app state
                temporal_client = AsyncMock()
                mock_handle = MagicMock()
                mock_handle.id = "mock-workflow-id"
                temporal_client.start_workflow.return_value = mock_handle
                app.state.temporal_client = temporal_client

                async with httpx.AsyncClient(
                    transport=httpx.ASGITransport(app=app), base_url="http://test"
                ) as client:
                    yield client


@pytest_asyncio.fixture
async def worker_light_client():
    try:
        async with httpx.AsyncClient(base_url=WORKER_LIGHT_URL, timeout=0.5) as client:
            resp = await client.get("/health")
            if resp.status_code == 200:
                yield client
                return
    except Exception:
        pass

    os.environ["IS_INTEGRATION_TEST"] = "true"
    with patch("worker_light.app.lifespan", side_effect=AsyncMock()):
        from worker_light.app import app

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            yield client


@pytest_asyncio.fixture
async def worker_heavy_client():
    try:
        async with httpx.AsyncClient(base_url=WORKER_HEAVY_URL, timeout=0.5) as client:
            resp = await client.get("/health")
            if resp.status_code == 200:
                yield client
                return
    except Exception:
        pass

    with patch("worker_heavy.app.lifespan", side_effect=AsyncMock()):
        from worker_heavy.app import app

        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://test"
        ) as client:
            yield client
