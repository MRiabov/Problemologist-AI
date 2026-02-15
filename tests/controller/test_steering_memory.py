import pytest
import pytest_asyncio
from unittest.mock import patch
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker

from controller.persistence.db import Base
from controller.persistence.steering_memory import (
    get_user_preferences,
    set_user_preference,
)
from controller.persistence.models import UserSteeringPreference

DATABASE_URL = "sqlite+aiosqlite:///:memory:"


@pytest_asyncio.fixture
async def test_session_factory():
    engine = create_async_engine(DATABASE_URL)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    factory = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)
    yield factory
    await engine.dispose()


@pytest.mark.asyncio
async def test_set_and_get_user_preference(test_session_factory):
    user_id = "user_123"
    pref_key = "theme"
    pref_value = {"color": "blue", "density": 0.5}

    # Patch get_sessionmaker to return our test factory
    with patch(
        "controller.persistence.steering_memory.get_sessionmaker",
        return_value=test_session_factory,
    ):
        # 1. Set preference
        await set_user_preference(user_id, pref_key, pref_value)

        # 2. Get preference and verify
        prefs = await get_user_preferences(user_id)
        assert pref_key in prefs
        assert prefs[pref_key] == pref_value

        # 3. Update preference
        new_value = {"color": "red"}
        await set_user_preference(user_id, pref_key, new_value)

        prefs = await get_user_preferences(user_id)
        assert prefs[pref_key] == new_value


@pytest.mark.asyncio
async def test_get_multiple_preferences(test_session_factory):
    user_id = "user_456"

    with patch(
        "controller.persistence.steering_memory.get_sessionmaker",
        return_value=test_session_factory,
    ):
        await set_user_preference(user_id, "p1", {"v": 1})
        await set_user_preference(user_id, "p2", {"v": 2})

        prefs = await get_user_preferences(user_id)
        assert len(prefs) == 2
        assert prefs["p1"] == {"v": 1}
        assert prefs["p2"] == {"v": 2}


@pytest.mark.asyncio
async def test_get_preferences_empty(test_session_factory):
    with patch(
        "controller.persistence.steering_memory.get_sessionmaker",
        return_value=test_session_factory,
    ):
        prefs = await get_user_preferences("non_existent_user")
        assert prefs == {}
