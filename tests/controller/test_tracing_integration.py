import asyncio
import uuid
from unittest.mock import patch

import pytest
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

from controller.observability.tracing import record_worker_events
from controller.persistence.db import Base
from controller.persistence.models import Episode, Trace
from shared.enums import EpisodeStatus, TraceType

DATABASE_URL = "sqlite+aiosqlite:///:memory:"


@pytest.mark.asyncio
async def test_record_worker_events():
    engine = create_async_engine(DATABASE_URL)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    session_factory = async_sessionmaker(
        bind=engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )

    episode_id = uuid.uuid4()

    with patch(
        "controller.observability.tracing.get_sessionmaker",
        return_value=session_factory,
    ):
        # Create a dummy episode first to satisfy foreign key constraint
        async with session_factory() as db:
            episode = Episode(
                id=episode_id, task="Test Task", status=EpisodeStatus.RUNNING
            )
            db.add(episode)
            await db.commit()

        try:
            events = [
                {"event_type": "test_event_1", "data": {"key": "val1"}},
                {"event_type": "test_event_2", "data": {"key": "val2"}},
            ]

            await record_worker_events(episode_id, events)

            # Verify in DB
            async with session_factory() as db:
                from sqlalchemy import select

                stmt = select(Trace).where(
                    Trace.episode_id == episode_id, Trace.trace_type == TraceType.EVENT
                )
                result = await db.execute(stmt)
                traces = result.scalars().all()

                assert len(traces) == 2, f"Expected 2 traces, found {len(traces)}"
                names = [t.name for t in traces]
                assert "test_event_1" in names
                assert "test_event_2" in names

                # Check metadata
                for t in traces:
                    assert t.metadata_vars["event_type"] in [
                        "test_event_1",
                        "test_event_2",
                    ]
        finally:
            await engine.dispose()


