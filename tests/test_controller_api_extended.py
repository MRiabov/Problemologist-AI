import uuid
from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from sqlalchemy.ext.asyncio import AsyncSession

from controller.api.main import app
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, ResponseStatus

client = TestClient(app)

def test_read_root():
    response = client.get("/")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "ok"
    assert data["service"] == "controller"

def test_health_extended():
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"

@patch("controller.api.main.temporal_client_instance", None)
def test_run_simulation_no_temporal():
    response = client.post(
        "/simulation/run", 
        json={"session_id": "test-session", "compound_json": "{}"}
    )
    assert response.status_code == 200
    assert response.json()["status"] == "error"
    assert "Temporal client not connected" in response.json()["message"]

@patch("controller.api.main.get_sessionmaker")
def test_run_agent_db_error(mock_get_sessionmaker):
    # Mock session factory to raise an error
    mock_session_factory = MagicMock()
    mock_session_factory.return_value.__aenter__.side_effect = Exception("DB Connection Failed")
    mock_get_sessionmaker.return_value = mock_session_factory

    with pytest.raises(Exception, match="DB Connection Failed"):
        client.post(
            "/agent/run", 
            json={"task": "test task", "session_id": "test-session"}
        )

@patch("controller.api.routes.skills.Path")
def test_list_skills_empty(mock_path):
    mock_path.return_value.exists.return_value = False
    
    response = client.get("/skills/")
    assert response.status_code == 200
    assert response.json() == []

@patch("controller.api.routes.skills.Path")
def test_list_skills_success(mock_path):
    mock_path.return_value.exists.return_value = True
    
    mock_item = MagicMock()
    mock_item.is_dir.return_value = True
    mock_item.name = "test-skill"
    
    mock_path.return_value.iterdir.return_value = [mock_item]
    
    response = client.get("/skills/")
    assert response.status_code == 200
    data = response.json()
    assert len(data) == 1
    assert data[0]["name"] == "test-skill"
    assert "Skill from test-skill" in data[0]["description"]

def test_trigger_backup_unauthorized():
    response = client.post("/ops/backup")
    assert response.status_code == 403
    assert response.json()["detail"] == "Invalid backup secret"

def test_trigger_backup_invalid_secret():
    response = client.post("/ops/backup", headers={"x-backup-secret": "wrong"})
    assert response.status_code == 403
    assert response.json()["detail"] == "Invalid backup secret"

@patch("controller.api.ops.os.getenv")
def test_trigger_backup_no_temporal(mock_getenv):
    mock_getenv.return_value = "secret"
    # We need to bypass the secret check
    with patch("controller.api.ops.BACKUP_SECRET", "secret"):
        response = client.post("/ops/backup", headers={"x-backup-secret": "secret"})
        # It should fail because request.app.state.temporal_client is missing in TestClient if not set
        assert response.status_code == 500
        assert "Temporal client not initialized" in response.json()["detail"]

@patch("controller.api.ops.os.getenv")
@patch("controller.api.ops.BackupWorkflow")
def test_trigger_backup_success(mock_workflow, mock_getenv):
    mock_getenv.side_effect = lambda k, default=None: "secret" if k == "BACKUP_SECRET" else "config-val"
    
    mock_client = AsyncMock()
    mock_handle = MagicMock()
    mock_handle.id = "test-workflow-id"
    mock_client.start_workflow.return_value = mock_handle
    
    app.state.temporal_client = mock_client
    
    with patch("controller.api.ops.BACKUP_SECRET", "secret"):
        response = client.post("/ops/backup", headers={"x-backup-secret": "secret"})
        assert response.status_code == 202
        assert response.json()["workflow_id"] == "test-workflow-id"
        assert response.json()["status"] == "Accepted"

def test_websocket_manager():
    from controller.api.manager import ConnectionManager
    manager = ConnectionManager()
    
    mock_ws = AsyncMock()
    episode_id = uuid.uuid4()
    
    # Test connect
    # manager.connect(episode_id, mock_ws) is async
    import asyncio
    asyncio.run(manager.connect(episode_id, mock_ws))
    assert episode_id in manager.active_connections
    assert mock_ws in manager.active_connections[episode_id]
    
    # Test broadcast
    asyncio.run(manager.broadcast(episode_id, {"type": "test"}))
    mock_ws.send_json.assert_called_with({"type": "test"})
    
    # Test disconnect
    manager.disconnect(episode_id, mock_ws)
    assert episode_id not in manager.active_connections

def test_websocket_broadcast_failure():
    from controller.api.manager import ConnectionManager
    manager = ConnectionManager()
    
    mock_ws = AsyncMock()
    mock_ws.send_json.side_effect = Exception("Send failed")
    episode_id = uuid.uuid4()
    
    import asyncio
    asyncio.run(manager.connect(episode_id, mock_ws))
    
    # Should handle failure and disconnect
    asyncio.run(manager.broadcast(episode_id, {"type": "fail"}))
    assert episode_id not in manager.active_connections
