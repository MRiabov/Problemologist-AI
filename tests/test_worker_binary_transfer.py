from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from worker.app import app
from worker.filesystem.router import FilesystemRouter

client = TestClient(app)


@patch("worker.api.routes.create_filesystem_router")
def test_fs_upload_file_binary(mock_create_router):
    """Test uploading a binary file via the new endpoint."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.local_backend = MagicMock()
    mock_create_router.return_value = mock_router

    # Mock read-only check
    mock_router._is_read_only.return_value = False

    file_content = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR"  # Partial PNG header

    response = client.post(
        "/fs/upload_file",
        data={"path": "/image.png"},
        files={"file": ("image.png", file_content, "image/png")},
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    assert response.json()["status"] == "success"

    # Verify mock call - note that upload_files takes a list of tuples
    mock_router.local_backend.upload_files.assert_called_with([("/image.png", file_content)])
    mock_router._is_read_only.assert_called_with("/image.png")


@patch("worker.api.routes.create_filesystem_router")
def test_fs_upload_file_read_only(mock_create_router):
    """Test uploading to a read-only path."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.local_backend = MagicMock()
    mock_create_router.return_value = mock_router

    # Mock read-only check
    mock_router._is_read_only.return_value = True

    file_content = b"content"

    response = client.post(
        "/fs/upload_file",
        data={"path": "/skills/test.py"},
        files={"file": ("test.py", file_content)},
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 403
    assert "read-only" in response.json()["detail"]
    mock_router.local_backend.upload_files.assert_not_called()


@patch("worker.api.routes.create_filesystem_router")
def test_get_asset_binary(mock_create_router):
    """Test downloading a binary file via the assets endpoint."""
    mock_router = MagicMock(spec=FilesystemRouter)
    mock_create_router.return_value = mock_router

    file_content = b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR"
    # Mock fs_router.read to return bytes
    mock_router.read.return_value = file_content
    # Mock exists check (needed for .glb/.stl logic in route, though we test png here)
    mock_router.exists.return_value = True

    response = client.get(
        "/assets/image.png",
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200
    assert response.content == file_content
    assert response.headers["content-type"] == "application/octet-stream"

    mock_router.read.assert_called_with("image.png")
