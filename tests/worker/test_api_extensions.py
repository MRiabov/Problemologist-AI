import pytest
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from worker.api.routes import get_router
from worker.app import app
from worker.workbenches.models import (
    ManufacturingConfig,
    ManufacturingMethod,
    WorkbenchResult,
)
from worker.filesystem.backend import FileInfo
from deepagents.backends.protocol import FileUploadResponse

client = TestClient(app)


@pytest.fixture
def mock_fs_router():
    mock = MagicMock()
    app.dependency_overrides[get_router] = lambda: mock
    yield mock
    app.dependency_overrides.clear()


def test_api_glob(mock_fs_router):
    # Setup mock return value
    mock_fs_router.glob.return_value = [
        FileInfo(path="/test.py", name="test.py", is_dir=False, size=100)
    ]

    response = client.post("/fs/glob", json={"pattern": "*.py", "path": "/"})

    assert response.status_code == 200
    data = response.json()
    assert len(data) == 1
    assert data[0]["path"] == "/test.py"
    mock_fs_router.glob.assert_called_with("*.py", "/")


def test_upload_file(mock_fs_router):
    # Setup mock return value
    mock_fs_router.local_backend.upload_files.return_value = [
        FileUploadResponse(path="test.txt", error=None),
        FileUploadResponse(path="image.png", error=None),
    ]

    files = [
        ("files", ("test.txt", b"content", "text/plain")),
        ("files", ("image.png", b"binary", "image/png")),
    ]

    response = client.post("/fs/upload_file", files=files)

    assert response.status_code == 200
    assert response.json()["status"] == "success"

    # Verify backend call
    mock_fs_router.local_backend.upload_files.assert_called_once()
    call_args = mock_fs_router.local_backend.upload_files.call_args[0][0]
    # call_args is a list of tuples (filename, content)
    assert len(call_args) == 2
    assert call_args[0] == ("test.txt", b"content")
    assert call_args[1] == ("image.png", b"binary")


@patch("worker.api.routes._load_component")
@patch("worker.api.routes.validate_and_price")
def test_api_analyze(mock_validate, mock_load, mock_fs_router):
    # Setup mocks
    mock_load.return_value = MagicMock()
    mock_validate.return_value = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.0,
        violations=[],
        metadata={"dof_count": 0, "dof_warning": False},
    )

    # Prepare request data
    config = ManufacturingConfig()
    request_data = {
        "method": "cnc",
        "config": config.model_dump(),
        "script_path": "part.py",
    }

    response = client.post("/benchmark/analyze", json=request_data)

    assert response.status_code == 200
    data = response.json()
    assert data["result"]["is_manufacturable"] is True
    assert data["result"]["unit_cost"] == 10.0

    mock_load.assert_called_once()
    mock_validate.assert_called_once()
    args, kwargs = mock_validate.call_args
    assert kwargs["method"] == ManufacturingMethod.CNC
    # config is passed as object
    assert isinstance(kwargs["config"], ManufacturingConfig)
