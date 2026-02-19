from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.clients.worker import WorkerClient
from shared.workers.filesystem.backend import LocalFilesystemBackend
from shared.workers.filesystem.router import (
    FilesystemRouter,
)


@pytest.fixture
def mock_backend():
    backend = MagicMock(spec=LocalFilesystemBackend)
    backend.upload_files.return_value = []
    # backend.read usually returns string for text read, but router.read handles bytes if direct
    # actually router.read calls local_backend._resolve(...).read_bytes() or similar
    # Wait, router.read implementation:
    # return local_p.read_bytes()
    # It doesn't use local_backend.read()! It uses local_backend._resolve()

    # Mock _resolve to return a mock path
    mock_path = MagicMock()
    mock_path.exists.return_value = True
    mock_path.read_bytes.return_value = b"some binary content"
    backend._resolve.return_value = mock_path
    backend.exists.return_value = True

    return backend


@pytest.fixture
def router(mock_backend):
    router = FilesystemRouter(local_backend=mock_backend)
    # Configure read-only prefixes for test
    router.READ_ONLY_PREFIXES = ("/readonly/",)
    return router


def test_router_upload_files_binary(router, mock_backend):
    files = [
        ("/workspace/test.bin", b"\x00\x01\x02"),
        ("/readonly/test.bin", b"\x03\x04\x05"),
    ]

    responses = router.upload_files(files)

    # Check backend call
    mock_backend.upload_files.assert_called_once()
    args, _ = mock_backend.upload_files.call_args
    uploaded = args[0]

    # Only the writable file should be passed to backend
    assert len(uploaded) == 1
    assert uploaded[0] == ("/workspace/test.bin", b"\x00\x01\x02")

    # Check responses
    # backend.upload_files returns [], so responses should contain only the error
    assert len(responses) == 1
    assert responses[0].path == "/readonly/test.bin"
    assert "read-only" in responses[0].error


@pytest.mark.asyncio
async def test_worker_client_upload_file():
    # We must patch where WorkerClient imports httpx, or mock the class used
    with patch("httpx.AsyncClient") as MockClient:
        mock_client_instance = AsyncMock()
        MockClient.return_value = mock_client_instance
        # Mock __aenter__ for context manager
        mock_client_instance.__aenter__.return_value = mock_client_instance
        mock_client_instance.__aexit__.return_value = None

        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.json.return_value = {"status": "success"}
        mock_client_instance.post.return_value = mock_response

        # When checking how WorkerClient uses httpx
        # It calls self._get_client() -> returns self.http_client or creates new
        # If creates new:
        # return httpx.AsyncClient()

        client = WorkerClient(base_url="http://test", session_id="sess")
        # Ensure we don't have a pre-existing client so it creates one using our mock

        success = await client.upload_file("/test.bin", b"\x00\xff")

        assert success is True
        mock_client_instance.post.assert_called_once()
        # call_args is a tuple (args, kwargs)
        # args[0] is URL
        args, kwargs = mock_client_instance.post.call_args
        assert args[0] == "http://test/fs/upload_file"
        assert kwargs["data"] == {"path": "/test.bin"}
        # Verify files argument structure
        assert "files" in kwargs
        assert kwargs["files"]["file"] == ("blob", b"\x00\xff")


@pytest.mark.asyncio
async def test_worker_client_read_file_binary():
    with patch("httpx.AsyncClient") as MockClient:
        mock_client_instance = AsyncMock()
        MockClient.return_value = mock_client_instance
        mock_client_instance.__aenter__.return_value = mock_client_instance
        mock_client_instance.__aexit__.return_value = None

        mock_response = MagicMock()
        mock_response.status_code = 200
        mock_response.content = b"\xde\xad\xbe\xef"
        mock_client_instance.post.return_value = mock_response

        client = WorkerClient(base_url="http://test", session_id="sess")
        content = await client.read_file_binary("/test.bin")

        assert content == b"\xde\xad\xbe\xef"
        mock_client_instance.post.assert_called_once()
        args, kwargs = mock_client_instance.post.call_args
        assert args[0] == "http://test/fs/read_blob"
        assert kwargs["json"] == {"path": "/test.bin"}
