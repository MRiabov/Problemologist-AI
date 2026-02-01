import os
import uuid
import pytest
from pathlib import Path
from src.dashboard.data import DashboardDataLayer
from src.environment.persistence import DatabaseManager, Episode, Step, Artifact

@pytest.fixture
def temp_db(tmp_path):
    db_path = tmp_path / "test_history.db"
    db_url = f"sqlite:///{db_path}"
    db_manager = DatabaseManager(db_url)
    db_manager.create_tables()
    
    # Seed dummy data
    session = db_manager.get_session()
    
    episode = Episode(problem_id="test_problem", status="success")
    session.add(episode)
    session.commit()
    session.refresh(episode)
    
    step = Step(
        episode_id=episode.id,
        sequence_index=0,
        tool_name="test_tool",
        tool_input="test_input",
        tool_output="test_output"
    )
    session.add(step)
    session.commit()
    session.refresh(step)
    
    artifact = Artifact(
        step_id=step.id,
        artifact_type="mesh",
        file_path="artifacts/test.stl"
    )
    session.add(artifact)
    session.commit()
    
    session.close()
    return db_path

def test_data_layer_fetching(temp_db):
    dal = DashboardDataLayer(str(temp_db))
    
    # Test get_all_episodes
    episodes = dal.get_all_episodes()
    assert len(episodes) == 1
    assert episodes[0].problem_id == "test_problem"
    
    # Test get_episode_by_id
    ep_id = episodes[0].id
    episode = dal.get_episode_by_id(ep_id)
    assert episode is not None
    assert len(episode.steps) == 1
    assert episode.steps[0].tool_name == "test_tool"
    assert len(episode.steps[0].artifacts) == 1
    assert episode.steps[0].artifacts[0].file_path == "artifacts/test.stl"
    
    # Test get_step_artifacts
    step_id = episode.steps[0].id
    artifacts = dal.get_step_artifacts(step_id)
    assert len(artifacts) == 1
    assert artifacts[0].artifact_type == "mesh"
