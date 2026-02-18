import asyncio
from unittest.mock import AsyncMock, MagicMock
import pytest
from controller.clients.backend import RemoteFilesystemBackend
from shared.backend.protocol import FileDownloadResponse, FileUploadResponse

@pytest.mark.asyncio
async def test_adownload_files_concurrent():
    mock_client = AsyncMock()

    async def slow_read_binary(path):
        if path == "error.txt":
            raise Exception("Read error")
        return b"content of " + path.encode()

    mock_client.read_file_binary.side_effect = slow_read_binary

    backend = RemoteFilesystemBackend(mock_client)
    paths = ["file1.txt", "file2.txt", "error.txt"]

    responses = await backend.adownload_files(paths)

    assert len(responses) == 3

    assert responses[0].path == "file1.txt"
    assert responses[0].content == b"content of file1.txt"
    assert responses[0].error is None

    assert responses[1].path == "file2.txt"
    assert responses[1].content == b"content of file2.txt"
    assert responses[1].error is None

    assert responses[2].path == "error.txt"
    assert responses[2].content is None
    assert "Read error" in responses[2].error

@pytest.mark.asyncio
async def test_aupload_files_concurrent():
    mock_client = AsyncMock()

    async def slow_upload_file(path, content):
        if path == "fail.txt":
            return False
        if path == "error.txt":
            raise Exception("Upload error")
        return True

    mock_client.upload_file.side_effect = slow_upload_file

    backend = RemoteFilesystemBackend(mock_client)
    files = [
        ("success.txt", b"data"),
        ("fail.txt", b"data"),
        ("error.txt", b"data"),
    ]

    responses = await backend.aupload_files(files)

    assert len(responses) == 3

    assert responses[0].path == "success.txt"
    assert responses[0].error is None

    assert responses[1].path == "fail.txt"
    assert responses[1].error == "failed"

    assert responses[2].path == "error.txt"
    assert "Upload error" in responses[2].error
