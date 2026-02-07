import pytest
import pytest_asyncio
import uuid
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from src.controller.persistence.db import Base
from src.controller.persistence.models import Episode, Asset
from src.shared.enums import EpisodeStatus, AssetType

DATABASE_URL = "sqlite+aiosqlite:///:memory:"

@pytest_asyncio.fixture
async def db_session():
    engine = create_async_engine(DATABASE_URL)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    
    async_session = sessionmaker(
        engine, class_=AsyncSession, expire_on_commit=False
    )
    
    async with async_session() as session:
        yield session
    
    await engine.dispose()

@pytest.mark.asyncio
async def test_asset_content_persistence(db_session):
    # Create an episode
    episode = Episode(
        id=uuid.uuid4(),
        task="test task",
        status=EpisodeStatus.COMPLETED
    )
    db_session.add(episode)
    await db_session.commit()
    
    # Create an asset with content
    code_content = "print('hello world')"
    asset = Asset(
        episode_id=episode.id,
        asset_type=AssetType.PYTHON,
        s3_path="path/to/script.py",
        content=code_content
    )
    db_session.add(asset)
    await db_session.commit()
    
    # Retrieve and verify
    await db_session.refresh(asset)
    assert asset.content == code_content
    assert asset.asset_type == AssetType.PYTHON

@pytest.mark.asyncio
async def test_episode_relationships(db_session):
    episode_id = uuid.uuid4()
    episode = Episode(id=episode_id, task="task", status=EpisodeStatus.RUNNING)
    db_session.add(episode)
    
    asset1 = Asset(episode_id=episode_id, asset_type=AssetType.IMAGE, s3_path="img1.png")
    asset2 = Asset(episode_id=episode_id, asset_type=AssetType.VIDEO, s3_path="vid1.mp4")
    db_session.add(asset1)
    db_session.add(asset2)
    
    await db_session.commit()
    
    # Query with relationship
    from sqlalchemy import select
    from sqlalchemy.orm import selectinload
    
    stmt = select(Episode).where(Episode.id == episode_id).options(selectinload(Episode.assets))
    result = await db_session.execute(stmt)
    retrieved_ep = result.scalar_one()
    
    assert len(retrieved_ep.assets) == 2
    paths = [a.s3_path for a in retrieved_ep.assets]
    assert "img1.png" in paths
    assert "vid1.mp4" in paths