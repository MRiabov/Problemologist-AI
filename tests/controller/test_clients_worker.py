from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from controller.clients.worker import WorkerClient
from shared.simulation.schemas import SimulatorBackendType
from worker.api.schema import EditOp, ExecuteResponse
from worker.filesystem.backend import FileInfo


@pytest.fixture
def mock_httpx_client():
    with patch("httpx.AsyncClient") as mock_client_cls:
        mock_client = AsyncMock()
        mock_client_cls.return_value = mock_client
        mock_client.__aenter__.return_value = mock_client
        yield mock_client


@pytest.mark.asyncio
async def test_worker_client_init():
    client = WorkerClient("http://worker:8000/", "test-session")
    assert client.base_url == "http://worker:8000"
    assert client.headers == {"X-Session-ID": "test-session"}


@pytest.mark.asyncio
async def test_list_files(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = [
        {"name": "file1.txt", "is_dir": False, "path": "/file1.txt", "size": 100},
        {"name": "dir1", "is_dir": True, "path": "/dir1", "size": 0},
    ]
    mock_httpx_client.post.return_value = mock_response

    files = await client.list_files("/")

    assert len(files) == 2
    assert isinstance(files[0], FileInfo)
    assert files[0].name == "file1.txt"

    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/fs/ls",
        json={"path": "/"},
        headers={"X-Session-ID": "test-session"},
        timeout=10.0,
    )


@pytest.mark.asyncio
async def test_read_file(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {"content": "hello world"}
    mock_httpx_client.post.return_value = mock_response

    content = await client.read_file("test.txt")

    assert content == "hello world"
    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/fs/read",
        json={"path": "test.txt"},
        headers={"X-Session-ID": "test-session"},
        timeout=10.0,
    )


@pytest.mark.asyncio
async def test_write_file(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {"status": "success"}
    mock_httpx_client.post.return_value = mock_response

    result = await client.write_file("test.txt", "content")

    assert result is True
    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/fs/write",
        json={"path": "test.txt", "content": "content", "overwrite": True},
        headers={"X-Session-ID": "test-session"},
        timeout=10.0,
    )


@pytest.mark.asyncio
async def test_edit_file(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {"status": "success"}
    mock_httpx_client.post.return_value = mock_response

    edits = [EditOp(old_string="foo", new_string="bar")]
    result = await client.edit_file("test.txt", edits)

    assert result is True
    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/fs/edit",
        json={
            "path": "test.txt",
            "edits": [{"old_string": "foo", "new_string": "bar"}],
        },
        headers={"X-Session-ID": "test-session"},
        timeout=10.0,
    )


@pytest.mark.asyncio
async def test_execute_python(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "stdout": "output",
        "stderr": "",
        "exit_code": 0,
        "timed_out": False,
    }
    mock_httpx_client.post.return_value = mock_response

    response = await client.execute_python("print('hello')", timeout=5)

    assert isinstance(response, ExecuteResponse)
    assert response.stdout == "output"
    assert response.exit_code == 0

    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/runtime/execute",
        json={"code": "print('hello')", "timeout": 5},
        headers={"X-Session-ID": "test-session"},
        timeout=10.0,
    )


@pytest.mark.asyncio
async def test_health_check(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {"status": "healthy"}
    mock_httpx_client.get.return_value = mock_response

    health = await client.get_health()
    assert health == {"status": "healthy"}

    mock_httpx_client.get.assert_called_once_with(
        "http://worker:8000/health", timeout=5.0
    )


@pytest.mark.asyncio
async def test_simulate(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "success": True,
        "message": "Simulation successful",
        "artifacts": {"render_paths": ["/render.mp4"]},
    }
    mock_httpx_client.post.return_value = mock_response

    result = await client.simulate("script.py")

    assert result.success is True
    assert result.message == "Simulation successful"
    assert result.artifacts["render_paths"] == ["/render.mp4"]

    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/benchmark/simulate",
        json={
            "script_path": "script.py",
            "backend": SimulatorBackendType.MUJOCO,
        },
        headers={"X-Session-ID": "test-session"},
        timeout=60.0,
    )


@pytest.mark.asyncio
async def test_validate(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "success": True,
        "message": "Validation successful",
    }
    mock_httpx_client.post.return_value = mock_response

    result = await client.validate("script.py")

    assert result.success is True

    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/benchmark/validate",
        json={"script_path": "script.py"},
        headers={"X-Session-ID": "test-session"},
        timeout=30.0,
    )


@pytest.mark.asyncio
async def test_submit(mock_httpx_client):
    client = WorkerClient("http://worker:8000", "test-session")

    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.json.return_value = {
        "success": True,
        "message": "Submission successful",
    }
    mock_httpx_client.post.return_value = mock_response

    result = await client.submit("script.py")

    assert result.success is True

    mock_httpx_client.post.assert_called_once_with(
        "http://worker:8000/benchmark/submit",
        json={"script_path": "script.py"},
        headers={"X-Session-ID": "test-session"},
        timeout=30.0,
    )
