import pytest

from worker.filesystem.backend import (
    LocalFilesystemBackend,
)
from worker.filesystem.router import (
    AccessMode,
    FilesystemRouter,
    MountPoint,
    WritePermissionError,
)


def test_local_backend_operations(tmp_path):
    session_dir = tmp_path / "sessions"
    backend = LocalFilesystemBackend.create("session-123", base_dir=session_dir)

    # Test Write
    test_content = "hello world"
    backend.write("test.txt", test_content)
    assert (session_dir / "session-123" / "test.txt").read_text() == "hello world"

    # Test Exists
    assert backend.exists("test.txt") is True

    # Test Read
    content = backend.read("test.txt")
    assert "hello world" in content

    # Test Edit
    backend.edit("test.txt", "world", "universe")
    assert (session_dir / "session-123" / "test.txt").read_text() == "hello universe"

    # Test LS
    files = backend.ls("/")
    assert len(files) >= 1
    # LocalFilesystemBackend.create might add objectives.yaml automatically
    names = [f.name for f in files]
    assert "test.txt" in names


def test_local_backend_isolation(tmp_path):
    session_dir = tmp_path / "sessions"

    # Session 1
    backend1 = LocalFilesystemBackend.create("s1", base_dir=session_dir)
    backend1.write("file.txt", "content1")

    # Session 2
    backend2 = LocalFilesystemBackend.create("s2", base_dir=session_dir)
    backend2.write("file.txt", "content2")

    assert (session_dir / "s1" / "file.txt").read_text() == "content1"
    assert (session_dir / "s2" / "file.txt").read_text() == "content2"


def test_filesystem_router_logic(tmp_path):
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


def test_router_ls_merged(tmp_path):
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
