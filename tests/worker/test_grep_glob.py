import pytest
from pathlib import Path
from unittest.mock import MagicMock, patch
from worker.filesystem.router import FilesystemRouter, MountPoint, AccessMode
from worker.filesystem.backend import LocalFilesystemBackend, FileInfo
from worker.api.routes import router
from worker.api.schema import GrepMatchModel
from fastapi.testclient import TestClient
from fastapi import FastAPI

@pytest.fixture
def mock_local_backend():
    backend = MagicMock(spec=LocalFilesystemBackend)
    backend.ls.return_value = []
    return backend

def test_router_grep_local(mock_local_backend):
    router = FilesystemRouter(local_backend=mock_local_backend)

    mock_local_backend.grep_raw.return_value = [
        {"path": "/foo.py", "line": 1, "text": "foo"}
    ]

    matches = router.grep("foo", path="/")
    assert len(matches) == 1
    assert matches[0].path == "/foo.py"
    assert matches[0].text == "foo"

    mock_local_backend.grep_raw.assert_called_with("foo", "/", None)

def test_router_grep_mount(tmp_path):
    # Setup mount
    mount_dir = tmp_path / "utils"
    mount_dir.mkdir()
    (mount_dir / "tool.py").write_text("def tool(): pass")

    mount = MountPoint(
        virtual_prefix="/utils",
        local_path=mount_dir,
        access_mode=AccessMode.READ_ONLY
    )

    local_backend = LocalFilesystemBackend.create("sess", base_dir=tmp_path)
    router = FilesystemRouter(local_backend=local_backend, mount_points=[mount])

    # Grep in mount
    matches = router.grep("def tool", path="/utils")
    assert len(matches) == 1
    # Check path correction
    assert matches[0].path == "/utils/tool.py"
    assert matches[0].line == 1
    assert matches[0].text == "def tool(): pass"

def test_router_glob_mount(tmp_path):
    # Setup mount
    mount_dir = tmp_path / "utils"
    mount_dir.mkdir()
    (mount_dir / "tool.py").touch()

    mount = MountPoint(
        virtual_prefix="/utils",
        local_path=mount_dir,
        access_mode=AccessMode.READ_ONLY
    )

    local_backend = LocalFilesystemBackend.create("sess", base_dir=tmp_path)
    router = FilesystemRouter(local_backend=local_backend, mount_points=[mount])

    # Glob in mount
    files = router.glob("*.py", path="/utils")
    assert len(files) == 1
    assert files[0].path == "/utils/tool.py"

def test_api_grep():
    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    # We need to mock get_router dependency or use overrides
    from worker.api.routes import get_router

    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.grep.return_value = [GrepMatchModel(path="/foo.py", line=1, text="foo")]

    app.dependency_overrides[get_router] = lambda: mock_router

    resp = client.post("/fs/grep", json={"pattern": "foo", "path": "/"})
    assert resp.status_code == 200
    data = resp.json()
    assert len(data["matches"]) == 1
    assert data["matches"][0]["path"] == "/foo.py"

def test_api_glob():
    app = FastAPI()
    app.include_router(router)
    client = TestClient(app)

    from worker.api.routes import get_router

    mock_router = MagicMock(spec=FilesystemRouter)
    mock_router.glob.return_value = [FileInfo(path="/foo.py", name="foo.py", is_dir=False, size=10)]

    app.dependency_overrides[get_router] = lambda: mock_router

    resp = client.post("/fs/glob", json={"pattern": "*.py", "path": "/"})
    assert resp.status_code == 200
    data = resp.json()
    assert len(data) == 1
    assert data[0]["path"] == "/foo.py"
