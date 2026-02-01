import uuid

import pytest

from src.environment.persistence import DatabaseManager, Episode, Step


@pytest.fixture
def db_manager(tmp_path):
    db_file = tmp_path / "test_history.db"
    db_url = f"sqlite:///{db_file}"
    manager = DatabaseManager(db_url)
    manager.create_tables()
    return manager


def test_create_episode(db_manager):
    episode = db_manager.create_episode(problem_id="test_prob_1")
    assert isinstance(episode.id, uuid.UUID)
    assert episode.problem_id == "test_prob_1"
    assert episode.status == "started"


def test_log_step(db_manager):
    episode = db_manager.create_episode(problem_id="test_prob_2")
    step = db_manager.log_step(
        episode_id=episode.id,
        sequence_index=0,
        tool_name="write_script",
        tool_input='print("hello")',
    )
    assert step.episode_id == episode.id
    assert step.tool_name == "write_script"

    # Verify relationship
    session = db_manager.get_session()
    saved_episode = session.get(Episode, episode.id)
    assert len(saved_episode.steps) == 1
    assert saved_episode.steps[0].id == step.id
    session.close()


def test_save_artifact(db_manager):
    episode = db_manager.create_episode(problem_id="test_prob_3")
    step = db_manager.log_step(episode.id, 0, "preview", "{}")

    artifact = db_manager.save_artifact(
        step_id=step.id,
        artifact_type="image",
        file_path="/tmp/test.png",
        content_hash="abc",
    )

    assert artifact.step_id == step.id
    assert artifact.artifact_type == "image"

    # Verify relationship
    session = db_manager.get_session()
    saved_step = session.get(Step, step.id)
    assert len(saved_step.artifacts) == 1
    assert saved_step.artifacts[0].id == artifact.id
    session.close()


def test_update_status(db_manager):
    episode = db_manager.create_episode(problem_id="test_prob_4")
    db_manager.update_episode_status(episode.id, "success", {"score": 0.95})

    session = db_manager.get_session()
    updated_episode = session.get(Episode, episode.id)
    assert updated_episode.status == "success"
    assert updated_episode.result_metrics == {"score": 0.95}
    session.close()


def test_wal_mode(tmp_path):
    db_file = tmp_path / "wal_test.db"
    db_url = f"sqlite:///{db_file}"
    manager = DatabaseManager(db_url)

    # Connect and check journal mode
    with manager.engine.connect() as conn:
        result = conn.exec_driver_sql("PRAGMA journal_mode").fetchone()
        assert result[0].lower() == "wal"
