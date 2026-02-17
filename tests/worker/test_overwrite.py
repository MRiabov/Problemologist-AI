from unittest.mock import MagicMock

import s3fs

from worker.filesystem.backend import (
    LocalFilesystemBackend,
    SimpleSessionManager,
)


def test_local_backend_overwrite(tmp_path):
    session_dir = tmp_path / "sessions"
    backend = LocalFilesystemBackend.create("sess", base_dir=session_dir)

    # First write
    backend.write("test.txt", "content1")
    assert (backend.root / "test.txt").read_text() == "content1"

    # Second write without overwrite (should fail)
    res = backend.write("test.txt", "content2", overwrite=False)
    assert res.error is not None
    assert "already exists" in res.error
    assert "Use 'overwrite=True'" in res.error
    assert (backend.root / "test.txt").read_text() == "content1"

    # Third write with overwrite (should succeed)
    res = backend.write("test.txt", "content3", overwrite=True)
    assert res.error is None
    assert (backend.root / "test.txt").read_text() == "content3"


def test_sandbox_backend_overwrite():
    fs = MagicMock(spec=s3fs.S3FileSystem)
    storage = {}
    fs.exists.side_effect = lambda p: p in storage

    # Mock open for writing
    class MockFile:
        def __init__(self, p):
            self.p = p

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def write(self, data):
            storage[self.p] = data

    fs.open.side_effect = lambda p, m: MockFile(p)

    backend = SandboxFilesystemBackend(fs, "bucket", SimpleSessionManager("s1"))

    # First write
    backend.write("test.txt", "content1")
    assert storage["bucket/s1/test.txt"] == b"content1"

    # Second write without overwrite
    res = backend.write("test.txt", "content2", overwrite=False)
    assert res.error is not None
    assert "already exists" in res.error
    assert "Use 'overwrite=True'" in res.error

    # Third write with overwrite
    res = backend.write("test.txt", "content3", overwrite=True)
    assert res.error is None
    assert storage["bucket/s1/test.txt"] == b"content3"
