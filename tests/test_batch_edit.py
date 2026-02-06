import pytest

from src.worker.filesystem.backend import SandboxFilesystemBackend, SimpleSessionManager


class MockFile:
    def __init__(self, content=b""):
        self.content = content

    def read(self):
        return self.content

    def write(self, content):
        self.content = content

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

class MockS3FileSystem:
    def __init__(self):
        self.files = {} # path -> content
        self.read_count = 0
        self.write_count = 0

    def open(self, path, mode="rb"):
        if "w" in mode:
            self.write_count += 1
            file = MockFile()
            # Capture the write
            def write_hook(content):
                self.files[path] = content
            file.write = write_hook
            return file
        self.read_count += 1
        if path not in self.files:
            raise FileNotFoundError(path)
        return MockFile(self.files[path])

    def exists(self, path):
        return path in self.files

    def ls(self, _path, _detail=False):
        # minimal stub for backend instantiation if it calls ls
        # (it doesn't in create/init)
        return []

def test_batch_edit_success():
    mock_fs = MockS3FileSystem()
    mock_fs.files["bucket/session1/test.txt"] = b"line1\nline2\nline3"

    backend = SandboxFilesystemBackend(
        fs=mock_fs,
        bucket="bucket",
        session_manager=SimpleSessionManager("session1")
    )

    edits = [
        ("line1", "LINE1"),
        ("line2", "LINE2"),
    ]

    success_count, failed_edit = backend.batch_edit("test.txt", edits)

    assert success_count == 2
    assert failed_edit is None
    assert mock_fs.files["bucket/session1/test.txt"] == b"LINE1\nLINE2\nline3"

    # 1 read, 1 write
    assert mock_fs.read_count == 1
    assert mock_fs.write_count == 1

def test_batch_edit_partial_failure():
    mock_fs = MockS3FileSystem()
    mock_fs.files["bucket/session1/test.txt"] = b"line1\nline2\nline3"

    backend = SandboxFilesystemBackend(
        fs=mock_fs,
        bucket="bucket",
        session_manager=SimpleSessionManager("session1")
    )

    edits = [
        ("line1", "LINE1"), # Success
        ("missing", "repl"), # Fail
        ("line3", "LINE3"), # Should not be reached
    ]

    success_count, failed_edit = backend.batch_edit("test.txt", edits)

    assert success_count == 1
    assert failed_edit == ("missing", "repl")

    # Verify file was updated with the FIRST edit
    assert mock_fs.files["bucket/session1/test.txt"] == b"LINE1\nline2\nline3"

    # 1 read, 1 write
    assert mock_fs.read_count == 1
    assert mock_fs.write_count == 1

def test_batch_edit_all_failure():
    mock_fs = MockS3FileSystem()
    mock_fs.files["bucket/session1/test.txt"] = b"line1\nline2\nline3"

    backend = SandboxFilesystemBackend(
        fs=mock_fs,
        bucket="bucket",
        session_manager=SimpleSessionManager("session1")
    )

    edits = [
        ("missing", "repl"),
    ]

    success_count, failed_edit = backend.batch_edit("test.txt", edits)

    assert success_count == 0
    assert failed_edit == ("missing", "repl")

    # File unchanged
    assert mock_fs.files["bucket/session1/test.txt"] == b"line1\nline2\nline3"

    # 1 read, 0 writes
    assert mock_fs.read_count == 1
    assert mock_fs.write_count == 0

def test_batch_edit_file_not_found():
    mock_fs = MockS3FileSystem()

    backend = SandboxFilesystemBackend(
        fs=mock_fs,
        bucket="bucket",
        session_manager=SimpleSessionManager("session1")
    )

    with pytest.raises(FileNotFoundError):
        backend.batch_edit("missing.txt", [("a", "b")])

    assert mock_fs.read_count == 1
    assert mock_fs.write_count == 0
