import structlog
from sqlalchemy import select
from .db import get_sessionmaker
from .models import UserSteeringPreference

logger = structlog.get_logger(__name__)


async def get_user_preferences(user_id: str) -> dict:
    """Retrieve all steering preferences for a given user."""
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        stmt = select(UserSteeringPreference).where(
            UserSteeringPreference.user_id == user_id
        )
        result = await db.execute(stmt)
        prefs = result.scalars().all()
        return {p.preference_key: p.preference_value for p in prefs}


async def set_user_preference(user_id: str, key: str, value: dict):
    """Upsert a steering preference for a given user."""
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        stmt = select(UserSteeringPreference).where(
            UserSteeringPreference.user_id == user_id,
            UserSteeringPreference.preference_key == key,
        )
        result = await db.execute(stmt)
        pref = result.scalar_one_or_none()

        if pref:
            pref.preference_value = value
            logger.info("preference_updated", user_id=user_id, key=key)
        else:
            pref = UserSteeringPreference(
                user_id=user_id, preference_key=key, preference_value=value
            )
            db.add(pref)
            logger.info("preference_created", user_id=user_id, key=key)

        await db.commit()
