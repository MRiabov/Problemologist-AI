import uuid
from unittest.mock import MagicMock, patch

import pytest
import pytest_asyncio
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker

from controller.agent.benchmark.storage import BenchmarkStorage
from controller.persistence.db import Base
from controller.persistence.models import BenchmarkAsset as BenchmarkAssetModel

DATABASE_URL = "sqlite+aiosqlite:///:memory:"


@pytest_asyncio.fixture
async def db_session():
    engine = create_async_engine(DATABASE_URL)
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    async_session = sessionmaker(
        bind=engine, class_=AsyncSession, expire_on_commit=False
    )

    async with async_session() as session:
        yield session

    await engine.dispose()


@pytest.fixture
def mock_boto3():
    with patch("controller.agent.benchmark.storage.boto3") as mock:
        mock_client = MagicMock()
        mock.client.return_value = mock_client
        yield mock_client


@pytest.mark.asyncio
async def test_save_asset_with_variants(db_session, mock_boto3):
    # Setup mocks for environment variables to avoid real AWS calls
    with patch.dict(
        "os.environ",
        {
            "S3_ENDPOINT_URL": "http://localhost:19000",
            "AWS_ACCESS_KEY_ID": "test",
            "AWS_SECRET_ACCESS_KEY": "test",
            "BENCHMARK_SOURCE_BUCKET": "source",
            "BENCHMARK_ASSETS_BUCKET": "assets",
        },
    ):
        storage = BenchmarkStorage()

        from shared.simulation.schemas import AssetMetadata

        benchmark_id = uuid.uuid4()
        script = "print('hello')"
        mjcf = "<xml></xml>"
        images = [b"image1", b"image2"]
        metadata = AssetMetadata(additional_info={"difficulty_score": 0.5})
        random_variants = [uuid.uuid4(), uuid.uuid4()]

        asset = await storage.save_asset(
            benchmark_id=benchmark_id,
            script=script,
            mjcf=mjcf,
            images=images,
            metadata=metadata,
            db=db_session,
            random_variants=random_variants,
        )

        assert asset.benchmark_id == benchmark_id
        assert asset.random_variants == random_variants

        # Verify DB persistence
        result = await db_session.get(BenchmarkAssetModel, benchmark_id)
        assert result is not None

        # Check variants in DB
        # SQLAlchemy JSON type might return list of strings for UUIDs when using SQLite
        db_variants = result.random_variants
        assert len(db_variants) == 2
        # Convert UUIDs to strings for comparison
        expected_variants = [str(v) for v in random_variants]
        actual_variants = [str(v) for v in db_variants]
        assert set(actual_variants) == set(expected_variants)
