import pytest

from controller.persistence.models import Asset, Episode, Trace
from shared.logging import configure_logging, get_logger


def test_logging_configuration():
    # Should not raise
    configure_logging("test_service")
    logger = get_logger("test")
    logger.info("test message", extra_field="value")


def test_models_instantiation():
    episode = Episode(task="test task", status="running", skill_git_hash="abc")
    assert episode.task == "test task"
    assert episode.status == "running"

    trace = Trace(episode=episode, langfuse_trace_id="trace_123")
    assert trace.episode == episode

    asset = Asset(episode=episode, asset_type="video", s3_path="s3://bucket/path")
    assert asset.episode == episode
    assert asset.asset_type == "video"


@pytest.mark.asyncio
async def test_db_setup():
    from controller.persistence.db import Base

    # We just check if we can get the metadata
    assert Base.metadata is not None
    assert "episodes" in Base.metadata.tables
    assert "traces" in Base.metadata.tables
    assert "assets" in Base.metadata.tables
