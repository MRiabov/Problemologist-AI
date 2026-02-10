import asyncio
import uuid
from controller.observability.tracing import record_worker_events
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Trace, Episode
from shared.enums import TraceType, EpisodeStatus


async def test_record_worker_events():
    episode_id = uuid.uuid4()
    session_factory = get_sessionmaker()

    # Create a dummy episode first to satisfy foreign key constraint
    async with session_factory() as db:
        episode = Episode(id=episode_id, task="Test Task", status=EpisodeStatus.RUNNING)
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

            assert len(traces) == 2
            names = [t.name for t in traces]
            assert "test_event_1" in names
            assert "test_event_2" in names

            # Check metadata
            for t in traces:
                assert t.metadata_vars["event_type"] in ["test_event_1", "test_event_2"]
    finally:
        # Cleanup
        async with session_factory() as db:
            from sqlalchemy import delete

            await db.execute(delete(Trace).where(Trace.episode_id == episode_id))
            await db.execute(delete(Episode).where(Episode.id == episode_id))
            await db.commit()


if __name__ == "__main__":
    try:
        asyncio.run(test_record_worker_events())
        print("Integration test passed!")
    except Exception as e:
        print(f"Integration test failed: {e}")
        import traceback

        traceback.print_exc()
        exit(1)
