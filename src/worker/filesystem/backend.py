"""SandboxFilesystemBackend implementation for S3-backed file operations.

This module provides a filesystem abstraction that maps virtual paths
to S3 storage with session-based isolation.
"""

from dataclasses import dataclass
from pathlib import PurePosixPath
from typing import Protocol

import s3fs
import structlog
from pydantic import BaseModel

from .db import S3Config, get_s3_filesystem

logger = structlog.get_logger(__name__)


class FileInfo(BaseModel):
    """Information about a file or directory."""

    path: str
    name: str
    is_dir: bool
    size: int | None = None


class SessionManager(Protocol):
    """Protocol for session management."""

    def get_session_id(self) -> str:
        """Return the current session ID."""
        ...


@dataclass
class SimpleSessionManager:
    """Simple session manager with a fixed session ID."""

    session_id: str

    def get_session_id(self) -> str:
        """Return the session ID."""
        return self.session_id


class SandboxFilesystemBackend:
    """S3-backed filesystem with session-based path isolation.

    Maps virtual paths (e.g., `/hello.txt`) to S3 paths
    (e.g., `s3://bucket/session_id/hello.txt`).
    """

    def __init__(
        self,
        fs: s3fs.S3FileSystem,
        bucket: str,
        session_manager: SessionManager,
    ) -> None:
        """Initialize the filesystem backend.

        Args:
            fs: S3FileSystem instance for storage operations.
            bucket: S3 bucket name.
            session_manager: Manager that provides the current session ID.
        """
        self._fs = fs
        self._bucket = bucket
        self._session_manager = session_manager

    @classmethod
    def create(
        cls,
        session_id: str,
        config: S3Config | None = None,
    ) -> "SandboxFilesystemBackend":
        """Factory method to create a new backend instance.

        Args:
            session_id: Session ID for path isolation.
            config: Optional S3 configuration.

        Returns:
            Configured SandboxFilesystemBackend instance.
        """
        if config is None:
            config = S3Config()

        fs = get_s3_filesystem(config)
        session_manager = SimpleSessionManager(session_id=session_id)

        return cls(fs=fs, bucket=config.s3_bucket, session_manager=session_manager)

    def _resolve_path(self, virtual_path: str) -> str:
        """Resolve a virtual path to an S3 path.

        Args:
            virtual_path: Virtual path (e.g., `/hello.txt` or `hello.txt`).

        Returns:
            Full S3 path (e.g., `bucket/session_id/hello.txt`).
        """
        session_id = self._session_manager.get_session_id()
        # Normalize the path: remove leading slashes
        normalized = virtual_path.lstrip("/")
        return f"{self._bucket}/{session_id}/{normalized}"

    def _virtual_path(self, s3_path: str) -> str:
        """Convert an S3 path back to a virtual path.

        Args:
            s3_path: Full S3 path.

        Returns:
            Virtual path relative to session root.
        """
        session_id = self._session_manager.get_session_id()
        prefix = f"{self._bucket}/{session_id}/"
        if s3_path.startswith(prefix):
            return "/" + s3_path[len(prefix) :]
        return "/" + s3_path.split("/")[-1]

    def ls(self, path: str = "/") -> list[FileInfo]:
        """List contents of a directory.

        Args:
            path: Virtual directory path.

        Returns:
            List of FileInfo objects for directory contents.
        """
        s3_path = self._resolve_path(path)
        logger.debug("filesystem_ls", virtual_path=path, s3_path=s3_path)

        try:
            # Ensure path ends with / for directory listing
            if not s3_path.endswith("/"):
                s3_path += "/"

            entries = self._fs.ls(s3_path, detail=True)
            result = []

            for entry in entries:
                name = PurePosixPath(entry["name"]).name
                virtual = self._virtual_path(entry["name"])
                is_dir = entry["type"] == "directory"
                size = entry.get("size") if not is_dir else None

                result.append(
                    FileInfo(path=virtual, name=name, is_dir=is_dir, size=size)
                )

            return result
        except FileNotFoundError:
            logger.debug("filesystem_ls_empty", virtual_path=path)
            return []

    def read(self, path: str) -> bytes:
        """Read file contents.

        Args:
            path: Virtual file path.

        Returns:
            File contents as bytes.

        Raises:
            FileNotFoundError: If file does not exist.
        """
        s3_path = self._resolve_path(path)
        logger.debug("filesystem_read", virtual_path=path, s3_path=s3_path)

        try:
            with self._fs.open(s3_path, "rb") as f:
                return f.read()
        except FileNotFoundError:
            logger.warning("filesystem_read_not_found", virtual_path=path)
            raise

    def write(self, path: str, content: bytes | str) -> None:
        """Write content to a file.

        Args:
            path: Virtual file path.
            content: Content to write (bytes or string).
        """
        s3_path = self._resolve_path(path)
        logger.debug(
            "filesystem_write",
            virtual_path=path,
            s3_path=s3_path,
            content_size=len(content),
        )

        # Convert string to bytes if needed
        if isinstance(content, str):
            content = content.encode("utf-8")

        with self._fs.open(s3_path, "wb") as f:
            f.write(content)

        logger.info("filesystem_write_complete", virtual_path=path)

    def edit(
        self,
        path: str,
        old_content: str,
        new_content: str,
    ) -> bool:
        """Edit file by replacing content.

        Args:
            path: Virtual file path.
            old_content: Content to find and replace.
            new_content: Replacement content.

        Returns:
            True if replacement was made, False if old_content not found.

        Raises:
            FileNotFoundError: If file does not exist.
        """
        logger.debug("filesystem_edit", virtual_path=path)

        current = self.read(path).decode("utf-8")

        if old_content not in current:
            logger.warning("filesystem_edit_not_found", virtual_path=path)
            return False

        updated = current.replace(old_content, new_content, 1)
        self.write(path, updated)

        logger.info("filesystem_edit_complete", virtual_path=path)
        return True

    def batch_edit(
        self,
        path: str,
        edits: list[tuple[str, str]],
    ) -> bool:
        """Edit file by applying multiple replacements sequentially.

        Args:
            path: Virtual file path.
            edits: List of (old_content, new_content) tuples.

        Returns:
            True if all replacements were made, False if any old_content not found.
            If False, no changes are written to the file.

        Raises:
            FileNotFoundError: If file does not exist.
        """
        logger.debug("filesystem_batch_edit", virtual_path=path, num_edits=len(edits))

        current = self.read(path).decode("utf-8")
        original = current

        for old_content, new_content in edits:
            if old_content not in current:
                logger.warning(
                    "filesystem_batch_edit_not_found",
                    virtual_path=path,
                    missing_content=old_content[:50],
                )
                return False
            current = current.replace(old_content, new_content, 1)

        if current != original:
            self.write(path, current)
            logger.info("filesystem_batch_edit_complete", virtual_path=path)

        return True

    def exists(self, path: str) -> bool:
        """Check if a path exists.

        Args:
            path: Virtual path to check.

        Returns:
            True if path exists, False otherwise.
        """
        s3_path = self._resolve_path(path)
        return self._fs.exists(s3_path)

    def delete(self, path: str) -> None:
        """Delete a file or directory.

        Args:
            path: Virtual path to delete.

        Raises:
            FileNotFoundError: If path does not exist.
        """
        s3_path = self._resolve_path(path)
        logger.debug("filesystem_delete", virtual_path=path, s3_path=s3_path)

        if not self._fs.exists(s3_path):
            raise FileNotFoundError(f"Path not found: {path}")

        self._fs.rm(s3_path, recursive=True)
        logger.info("filesystem_delete_complete", virtual_path=path)
