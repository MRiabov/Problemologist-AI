import pytest
from unittest.mock import MagicMock
from src.worker.filesystem.backend import SandboxFilesystemBackend, SimpleSessionManager

class TestSandboxFilesystemBackend:
    @pytest.fixture
    def backend(self):
        mock_fs = MagicMock()
        session_manager = SimpleSessionManager("test-session")
        return SandboxFilesystemBackend(fs=mock_fs, bucket="test-bucket", session_manager=session_manager)

    def test_batch_edit_success(self, backend):
        # Mock read to return initial content
        backend.read = MagicMock(return_value=b"Line 1\nLine 2\nLine 3")
        backend.write = MagicMock()

        edits = [
            ("Line 1", "Modified 1"),
            ("Line 3", "Modified 3")
        ]

        success = backend.batch_edit("test.txt", edits)

        assert success is True
        backend.write.assert_called_once()
        args, _ = backend.write.call_args
        assert args[0] == "test.txt"
        assert args[1] == "Modified 1\nLine 2\nModified 3"

    def test_batch_edit_failure(self, backend):
        backend.read = MagicMock(return_value=b"Line 1\nLine 2\nLine 3")
        backend.write = MagicMock()

        edits = [
            ("Line 1", "Modified 1"),
            ("Line 4", "Modified 4") # Fails
        ]

        success = backend.batch_edit("test.txt", edits)

        assert success is False
        backend.write.assert_not_called()

    def test_batch_edit_no_changes(self, backend):
        backend.read = MagicMock(return_value=b"Line 1\nLine 2\nLine 3")
        backend.write = MagicMock()

        # Edit that replaces with same content effectively (or if I passed same content)
        # But here let's say edit found but no change?
        # replace() always returns a new string.
        # If I pass correct old content, it will replace.

        # Test case: Valid edit but results in same content?
        # e.g. "Line 1" -> "Line 1"
        edits = [("Line 1", "Line 1")]

        success = backend.batch_edit("test.txt", edits)

        assert success is True
        # Logic says: if current != original: write.
        # Here current == original, so NO write.
        backend.write.assert_not_called()
