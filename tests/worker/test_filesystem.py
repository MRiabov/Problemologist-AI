import pytest
import io
import s3fs
from unittest.mock import MagicMock, patch
from worker.filesystem.backend import (
    SandboxFilesystemBackend,
    LocalFilesystemBackend,
    SimpleSessionManager,
    FileInfo,
)
from worker.filesystem.router import (
    FilesystemRouter,
    MountPoint,
    AccessMode,
    WritePermissionError,
)
from pathlib import Path


@pytest.fixture
def mock_fs():
    fs = MagicMock(spec=s3fs.S3FileSystem)
    # Simple in-memory storage simulation for the mock
    storage = {}

    def mock_open(path, mode="rb"):
        class MockFile:
            def __init__(self, p, m):
                self.p = p
                self.m = m
                self.buf = io.BytesIO(storage.get(p, b"")) if "r" in m else io.BytesIO()

            def read(self):
                return self.buf.read()

            def write(self, data):
                self.buf.write(data)

            def __enter__(self):
                return self

            def __exit__(self, *args):
                if "w" in self.m:
                    storage[self.p] = self.buf.getvalue()

        return MockFile(path, mode)

    fs.open.side_effect = mock_open
    fs.exists.side_effect = lambda p: p in storage
    fs.ls.side_effect = lambda p, detail=False: [
        {"name": k, "type": "file", "size": len(v)}
        for k, v in storage.items()
        if k.startswith(p.rstrip("/") + "/")
    ]

    return fs, storage


def test_sandbox_backend_operations(mock_fs):
    fs, storage = mock_fs
    session_id = "session-123"
    bucket = "test-bucket"
    manager = SimpleSessionManager(session_id)
    backend = SandboxFilesystemBackend(fs, bucket, manager)

    # Test Write
    test_content = "hello world"
    backend.write("test.txt", test_content)
    assert storage["test-bucket/session-123/test.txt"] == b"hello world"

    # Test Exists
    assert backend.exists("test.txt") is True

    # Test Read
    content = backend.read("test.txt")
    assert "hello world" in content

    # Test Edit
    backend.edit("test.txt", "world", "universe")
    assert storage["test-bucket/session-123/test.txt"] == b"hello universe"

    # Test LS
    # s3fs.ls returns paths including bucket
    files = backend.ls("/")
    assert len(files) == 1
    assert files[0].name == "test.txt"


def test_sandbox_backend_isolation(mock_fs):
    fs, storage = mock_fs
    bucket = "test-bucket"

    # Session 1
    backend1 = SandboxFilesystemBackend(fs, bucket, SimpleSessionManager("s1"))
    backend1.write("file.txt", "content1")

    # Session 2
    backend2 = SandboxFilesystemBackend(fs, bucket, SimpleSessionManager("s2"))
    backend2.write("file.txt", "content2")

    assert storage["test-bucket/s1/file.txt"] == b"content1"
    assert storage["test-bucket/s2/file.txt"] == b"content2"


def test_filesystem_router_logic(mock_fs, tmp_path):
    # Setup Local backend
    session_dir = tmp_path / "sessions"
    backend = LocalFilesystemBackend.create("sess", base_dir=session_dir)

    # Setup local mount
    local_utils = tmp_path / "utils"
    local_utils.mkdir()
    (local_utils / "tool.py").write_text("print('tool')")

    mounts = [
        MountPoint(
            virtual_prefix="/utils",
            local_path=local_utils,
            access_mode=AccessMode.READ_ONLY,
        )
    ]

    router = FilesystemRouter(local_backend=backend, mount_points=mounts)

    # Test reading from local mount
    content = router.read("/utils/tool.py")
    assert content == b"print('tool')"

    # Test reading from Local
    router.write("app.py", "print('app')")
    assert router.read("app.py") == b"print('app')"

    # Test write restriction
    with pytest.raises(WritePermissionError):
        router.write("/utils/new.py", "content")


def test_router_ls_merged(mock_fs, tmp_path):
    session_dir = tmp_path / "sessions"
    backend = LocalFilesystemBackend.create("sess", base_dir=session_dir)

    local_utils = tmp_path / "utils"
    local_utils.mkdir()
    (local_utils / "helper.py").write_text("# helper")

    mounts = [
        MountPoint(
            virtual_prefix="/utils",
            local_path=local_utils,
            access_mode=AccessMode.READ_ONLY,
        )
    ]

    router = FilesystemRouter(local_backend=backend, mount_points=mounts)
    router.write("main.py", "# main")

    # LS root
    root_files = router.ls("/")
    names = [f.name for f in root_files]
    assert "utils" in names
    assert "main.py" in names
