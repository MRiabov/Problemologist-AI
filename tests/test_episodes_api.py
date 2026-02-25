import uuid
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock

import pytest
from fastapi.testclient import TestClient

from controller.api.main import app
from controller.api.routes.episodes import get_db
from controller.persistence.models import Asset, Episode
from shared.enums import AssetType, EpisodeStatus


@pytest.fixture
def mock_db():
    session = AsyncMock()
    app.dependency_overrides[get_db] = lambda: session
    yield session
    app.dependency_overrides.clear()


client = TestClient(app)


def _create_mock_episode(episode_id, task="task", status=EpisodeStatus.COMPLETED):
    mock_episode = MagicMock(spec=Episode)
    mock_episode.id = episode_id
    mock_episode.task = task
    mock_episode.status = status
    mock_episode.created_at = datetime.utcnow()
    mock_episode.updated_at = datetime.utcnow()
    mock_episode.skill_git_hash = None
    mock_episode.template_versions = None
    mock_episode.metadata_vars = None
    mock_episode.todo_list = None
    mock_episode.journal = None
    mock_episode.plan = None
    mock_episode.traces = []
    mock_episode.assets = []
    return mock_episode


def test_get_episode_not_found(mock_db):
    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = None
    mock_db.execute.return_value = mock_result

    episode_id = uuid.uuid4()
    response = client.get(f"/episodes/{episode_id}")

    assert response.status_code == 404
    assert response.json()["detail"] == "Episode not found"


def test_get_episode_success(mock_db):
    episode_id = uuid.uuid4()
    mock_episode = _create_mock_episode(episode_id)

    mock_asset = MagicMock(spec=Asset)
    mock_asset.id = 1
    mock_asset.asset_type = AssetType.PYTHON
    mock_asset.s3_path = "test.py"
    mock_asset.content = "print(1)"
    mock_asset.created_at = datetime.utcnow()
    mock_episode.assets = [mock_asset]

    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = mock_episode
    mock_db.execute.return_value = mock_result

    response = client.get(f"/episodes/{episode_id}")

    assert response.status_code == 200
    data = response.json()
    assert data["id"] == str(episode_id)
    assert len(data["assets"]) == 1
    assert data["assets"][0]["asset_type"] == AssetType.PYTHON
    assert data["assets"][0]["s3_path"] == "test.py"


def test_list_episodes(mock_db):
    mock_episode = _create_mock_episode(
        uuid.uuid4(), task="task", status=EpisodeStatus.RUNNING
    )

    mock_result = MagicMock()
    mock_result.scalars.return_value.all.return_value = [mock_episode]
    mock_db.execute.return_value = mock_result

    response = client.get("/episodes/")

    assert response.status_code == 200
    assert len(response.json()) == 1
    assert response.json()[0]["task"] == "task"
    # Verify default limit and offset
    # Note: with SQLAlchemy 2.0 and AsyncSession, verifying calls on mock_db.execute can be complex
    # but we can check if it was called.


def test_list_episodes_pagination(mock_db):
    mock_result = MagicMock()
    mock_result.scalars.return_value.all.return_value = []
    mock_db.execute.return_value = mock_result

    response = client.get("/episodes/?limit=10&offset=5")

    assert response.status_code == 200

    # Verify that the database query actually applied the limit and offset
    assert mock_db.execute.called
    args, _ = mock_db.execute.call_args
    stmt = args[0]
    assert stmt._limit == 10
    assert stmt._offset == 5
