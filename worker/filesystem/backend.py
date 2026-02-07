"""SandboxFilesystemBackend implementation for S3-backed file operations.

This module provides a filesystem abstraction that maps virtual paths
to S3 storage with session-based isolation.
"""

import asyncio
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import PurePosixPath
from typing import Any, Protocol, runtime_checkable

import s3fs
import structlog
import wcmatch.glob as wcglob
from pydantic import BaseModel, StrictBool, StrictInt, StrictStr

from deepagents.backends.protocol import (
    BackendProtocol,
    EditResult,
    FileDownloadResponse,
    FileInfo as ProtocolFileInfo,
    FileUploadResponse,
    GrepMatch,
    WriteResult,
)
from deepagents.backends.utils import (
    check_empty_content,
    format_content_with_line_numbers,
    perform_string_replacement,
)
from shared.type_checking import type_check

from .db import S3Config, get_s3_filesystem

logger = structlog.get_logger(__name__)


class FileInfo(BaseModel):
    """Information about a file or directory."""

    path: StrictStr
    name: StrictStr
    is_dir: StrictBool
    size: StrictInt | None = None


@runtime_checkable
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


@type_check
class SandboxFilesystemBackend(BackendProtocol):
    """S3-backed filesystem with session-based path isolation.

    Maps virtual paths (e.g., `/hello.txt`) to S3 paths
    (e.g., `s3://bucket/session_id/hello.txt`).
    Implements BackendProtocol for compatibility with deepagents.
    """

    # FIXME: actually, this should also override the "BaseSandbox" or whatever
    # actual Sandbox backend in the deep agents implementation. However, I don't care.

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

    # --- BackendProtocol Implementation ---

    def ls_info(self, path: str) -> list[ProtocolFileInfo]:
        """List files and directories with metadata."""
        s3_path = self._resolve_path(path)
        try:
            if not s3_path.endswith("/"):
                s3_path += "/"

            entries = self._fs.ls(s3_path, detail=True)
            results = []
            for entry in entries:
                is_dir = entry["type"] == "directory"
                virt = self._virtual_path(entry["name"])
                if is_dir and not virt.endswith("/"):
                    virt += "/"

                info: ProtocolFileInfo = {
                    "path": virt,
                    "is_dir": is_dir,
                    "size": entry.get("size", 0) if not is_dir else 0,
                }
                # S3fs might provide LastModified
                if "LastModified" in entry:
                    info["modified_at"] = entry["LastModified"].isoformat()
                results.append(info)

            results.sort(key=lambda x: x["path"])
            return results
        except FileNotFoundError:
            return []

    def read(self, file_path: str, offset: int = 0, limit: int = 2000) -> str:
        """Read file content with line numbers."""
        s3_path = self._resolve_path(file_path)
        try:
            with self._fs.open(s3_path, "rb") as f:
                content = f.read().decode("utf-8")

            empty_msg = check_empty_content(content)
            if empty_msg:
                return empty_msg

            lines = content.splitlines()
            start_idx = offset
            end_idx = min(start_idx + limit, len(lines))

            if start_idx >= len(lines):
                return f"Error: Line offset {offset} exceeds file length ({len(lines)} lines)"

            selected_lines = lines[start_idx:end_idx]
            return format_content_with_line_numbers(
                selected_lines, start_line=start_idx + 1
            )
        except FileNotFoundError:
            return f"Error: File '{file_path}' not found"
        except Exception as e:
            return f"Error reading file '{file_path}': {e!s}"

    def write(self, file_path: str, content: str) -> WriteResult:
        """Write content to a new file."""
        s3_path = self._resolve_path(file_path)
        if self._fs.exists(s3_path):
            return WriteResult(
                error=f"Cannot write to {file_path} because it already exists."
            )

        try:
            with self._fs.open(s3_path, "wb") as f:
                f.write(content.encode("utf-8"))
            return WriteResult(path=file_path, files_update=None)
        except Exception as e:
            return WriteResult(error=f"Error writing file '{file_path}': {e!s}")

    def edit(
        self,
        file_path: str,
        old_string: str,
        new_string: str,
        replace_all: bool = False,
    ) -> EditResult:
        """Perform exact string replacements."""
        s3_path = self._resolve_path(file_path)
        if not self._fs.exists(s3_path):
            return EditResult(error=f"Error: File '{file_path}' not found")

        try:
            with self._fs.open(s3_path, "rb") as f:
                content = f.read().decode("utf-8")

            result = perform_string_replacement(
                content, old_string, new_string, replace_all
            )
            if isinstance(result, str):
                return EditResult(error=result)

            new_content, occurrences = result
            with self._fs.open(s3_path, "wb") as f:
                f.write(new_content.encode("utf-8"))

            return EditResult(
                path=file_path, files_update=None, occurrences=int(occurrences)
            )
        except Exception as e:
            return EditResult(error=f"Error editing file '{file_path}': {e!s}")

    def grep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str:
        """Literal text search in files. Fallback to Python search for S3."""
        # For simplicity, we implement a basic Python search over S3
        base_virt = path or "/"
        try:
            files = self.glob_info(glob or "**/*", path=base_virt)
        except Exception as e:
            return f"Error during grep: {e!s}"

        regex = re.compile(re.escape(pattern))
        matches: list[GrepMatch] = []

        for f_info in files:
            if f_info["is_dir"]:
                continue

            try:
                # We need a dedicated 'read_raw' or similar, but we'll use aread internal logic
                s3_p = self._resolve_path(f_info["path"])
                with self._fs.open(s3_p, "rb") as f:
                    content = f.read().decode("utf-8")

                for line_num, line in enumerate(content.splitlines(), 1):
                    if regex.search(line):
                        matches.append(
                            {"path": f_info["path"], "line": line_num, "text": line}
                        )
            except:
                continue

        return matches

    def glob_info(self, pattern: str, path: str = "/") -> list[ProtocolFileInfo]:
        """Find files matching a glob pattern."""
        # s3fs glob returns full paths
        s3_base = self._resolve_path(path)
        if not s3_base.endswith("/"):
            s3_base += "/"

        # Adjust pattern for s3fs
        full_pattern = s3_base + pattern.lstrip("/")
        try:
            matched = self._fs.glob(full_pattern, detail=True)
            results = []
            for name, entry in matched.items():
                is_dir = entry["type"] == "directory"
                virt = self._virtual_path(name)
                if is_dir and not virt.endswith("/"):
                    virt += "/"

                results.append(
                    {
                        "path": virt,
                        "is_dir": is_dir,
                        "size": entry.get("size", 0) if not is_dir else 0,
                    }
                )
            results.sort(key=lambda x: x["path"])
            return results
        except:
            return []

    def upload_files(self, files: list[tuple[str, bytes]]) -> list[FileUploadResponse]:
        """Upload multiple files."""
        responses = []
        for path, content in files:
            s3_p = self._resolve_path(path)
            try:
                with self._fs.open(s3_p, "wb") as f:
                    f.write(content)
                responses.append(FileUploadResponse(path=path, error=None))
            except Exception as e:
                responses.append(FileUploadResponse(path=path, error="invalid_path"))
        return responses

    def download_files(self, paths: list[str]) -> list[FileDownloadResponse]:
        """Download multiple files."""
        responses = []
        for path in paths:
            s3_p = self._resolve_path(path)
            try:
                with self._fs.open(s3_p, "rb") as f:
                    content = f.read()
                responses.append(
                    FileDownloadResponse(path=path, content=content, error=None)
                )
            except FileNotFoundError:
                responses.append(
                    FileDownloadResponse(path=path, error="file_not_found")
                )
            except Exception:
                responses.append(FileDownloadResponse(path=path, error="invalid_path"))
        return responses

    # --- Legacy methods for backward compatibility if needed ---

    def ls(self, path: str = "/") -> list[FileInfo]:
        """List contents of a directory (Legacy)."""
        infos = self.ls_info(path)
        return [
            FileInfo(
                path=i["path"],
                name=i["path"].rstrip("/").split("/")[-1] or "/",
                is_dir=i["is_dir"],
                size=i["size"],
            )
            for i in infos
        ]

    def exists(self, path: str) -> bool:
        """Check if a path exists."""
        s3_path = self._resolve_path(path)
        return self._fs.exists(s3_path)

    def delete(self, path: str) -> None:
        """Delete a file or directory."""
        s3_path = self._resolve_path(path)
        if not self._fs.exists(s3_path):
            raise FileNotFoundError(f"Path not found: {path}")
        self._fs.rm(s3_path, recursive=True)
