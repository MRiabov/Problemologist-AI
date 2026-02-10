from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from worker.app import app
from worker.filesystem.backend import FileInfo
from worker.filesystem.router import FilesystemRouter

client = TestClient(app)


def test_health():
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}


@patch("worker.api.routes.create_filesystem_router")
def test_fs_ls(mock_create_router):
    """Test listing files via API."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.ls.return_value = [
        FileInfo(path="/test.txt", name="test.txt", is_dir=False, size=10)
    ]
    mock_create_router.return_value = mock_router

    response = client.post(
        "/fs/ls", json={"path": "/"}, headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    assert len(response.json()) == 1
    assert response.json()[0]["name"] == "test.txt"
    mock_router.ls.assert_called_with("/")


@patch("worker.api.routes.create_filesystem_router")
def test_fs_read(mock_create_router):
    """Test reading a file via API."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.read.return_value = b"hello world"
    mock_create_router.return_value = mock_router

    response = client.post(
        "/fs/read", json={"path": "/test.txt"}, headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    assert response.json()["content"] == "hello world"
    mock_router.read.assert_called_with("/test.txt")


@patch("worker.api.routes.create_filesystem_router")
def test_fs_write(mock_create_router):
    """Test writing a file via API."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_create_router.return_value = mock_router

    response = client.post(
        "/fs/write",
        json={"path": "/test.txt", "content": "new content"},
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    assert response.json()["status"] == "success"
    mock_router.write.assert_called_with("/test.txt", "new content")


@patch("worker.api.routes.create_filesystem_router")
def test_fs_edit(mock_create_router):
    """Test editing a file via API."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.exists.return_value = True
    mock_router.edit.return_value = True
    mock_create_router.return_value = mock_router

    response = client.post(
        "/fs/edit",
        json={
            "path": "/test.txt",
            "edits": [{"old_string": "old", "new_string": "new"}],
        },
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    assert response.json()["status"] == "success"
    mock_router.edit.assert_called_with("/test.txt", "old", "new")


def test_execute_runtime():
    """Test executing Python code via API."""
    response = client.post(
        "/runtime/execute",
        json={"code": "print('hello from api')", "timeout": 5},
        headers={"X-Session-ID": "test-session"},
    )
    assert response.status_code == 200
    data = response.json()
    assert "hello from api" in data["stdout"]
    assert data["exit_code"] == 0
    assert data["timed_out"] is False


def test_execute_runtime_timeout():
    """Test runtime execution timeout via API."""
    response = client.post(
        "/runtime/execute",
        json={"code": "import time; time.sleep(2)", "timeout": 1},
        headers={"X-Session-ID": "test-session"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["timed_out"] is True
    assert data["exit_code"] == -1


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.simulate")
def test_benchmark_simulate(mock_simulate, mock_load, tmp_path):
    """Test the benchmark simulate endpoint."""
    mock_load.return_value = MagicMock()
    mock_simulate.return_value = MagicMock(
        success=True, summary="stable", render_paths=[], mjcf_content="<mjcf/>"
    )

    # Create a dummy events.jsonl to test collection
    events_file = tmp_path / "events.jsonl"
    events_file.write_text('{"event_type": "component_usage", "category": "test"}\n')

    with patch("worker.api.routes.create_filesystem_router") as mock_create_router:
        mock_router = MagicMock()
        mock_router.local_backend.root = tmp_path
        mock_router.exists.return_value = True
        mock_router.read.return_value = events_file.read_bytes()
        mock_create_router.return_value = mock_router

        response = client.post(
            "/benchmark/simulate",
            json={"script_path": "main.py"},
            headers={"X-Session-ID": "test-session"},
        )

        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert len(data["events"]) == 1
        assert data["events"][0]["event_type"] == "component_usage"


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.validate")
def test_benchmark_validate(mock_validate, mock_load, tmp_path):
    """Test the benchmark validate endpoint."""
    mock_load.return_value = MagicMock()
    mock_validate.return_value = True

    with patch("worker.api.routes.create_filesystem_router") as mock_create_router:
        mock_router = MagicMock()
        mock_router.local_backend.root = tmp_path
        mock_router.exists.return_value = False  # No events
        mock_create_router.return_value = mock_router

        response = client.post(
            "/benchmark/validate",
            json={"script_path": "main.py"},
            headers={"X-Session-ID": "test-session"},
        )

        assert response.status_code == 200
        data = response.json()
        assert data["success"] is True
        assert "events" in data
