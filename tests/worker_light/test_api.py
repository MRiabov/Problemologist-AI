from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from shared.workers.filesystem.backend import FileInfo
from shared.workers.filesystem.router import FilesystemRouter
from worker_light.app import app

client = TestClient(app)


def test_health():
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy"}


@patch("worker_light.api.routes.create_filesystem_router")
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


@patch("worker_light.api.routes.create_filesystem_router")
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


@patch("worker_light.api.routes.create_filesystem_router")
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
    mock_router.write.assert_called_with("/test.txt", "new content", overwrite=False)


@patch("worker_light.api.routes.create_filesystem_router")
def test_fs_exists(mock_create_router):
    """Test checking file existence via API."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.exists.return_value = True
    mock_create_router.return_value = mock_router

    response = client.post(
        "/fs/exists",
        json={"path": "/test.txt"},
        headers={"X-Session-ID": "test-session"},
    )

    assert response.status_code == 200
    assert response.json()["exists"] is True
    mock_router.exists.assert_called_with("/test.txt")


@patch("worker_light.api.routes.create_filesystem_router")
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
