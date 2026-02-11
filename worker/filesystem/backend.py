"""SandboxFilesystemBackend implementation for S3-backed file operations.

This module provides a filesystem abstraction that maps virtual paths
to S3 storage with session-based isolation.
"""

import re
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Protocol, runtime_checkable

import s3fs
import structlog
from deepagents.backends.protocol import (
    BackendProtocol,
    EditResult,
    FileDownloadResponse,
    FileUploadResponse,
    GrepMatch,
    WriteResult,
)
from deepagents.backends.protocol import (
    FileInfo as ProtocolFileInfo,
)
from deepagents.backends.utils import (
    check_empty_content,
    format_content_with_line_numbers,
    perform_string_replacement,
)
from pydantic import BaseModel, StrictBool, StrictInt, StrictStr

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
            except Exception:
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


@type_check
class LocalFilesystemBackend(BackendProtocol):
    """Local-disk backed filesystem with session-based isolation.

    Uses a local directory (e.g., /tmp/sessions/{session_id}) as the root.
    Implements BackendProtocol for compatibility with deepagents.
    """

    def __init__(
        self,
        root: Path,
        session_id: str,
        s3_backend: SandboxFilesystemBackend | None = None,
    ) -> None:
        """Initialize the local filesystem backend.

        Args:
            root: Local root directory for this session.
            session_id: Session ID for isolation.
            s3_backend: Optional S3 backend for synchronization.
        """
        self.root = root
        self.session_id = session_id
        self.s3_backend = s3_backend
        self.root.mkdir(parents=True, exist_ok=True)

    @classmethod
    def create(
        cls,
        session_id: str,
        base_dir: Path = Path("/tmp/sessions"),
        s3_config: S3Config | None = None,
    ) -> "LocalFilesystemBackend":
        """Factory method to create a new local backend instance."""
        root = base_dir / session_id
        s3_backend = SandboxFilesystemBackend.create(session_id, s3_config)
        backend = cls(root=root, session_id=session_id, s3_backend=s3_backend)
        try:
            from worker.objectives_template import ensure_objectives_yaml

            ensure_objectives_yaml(backend.root)
        except Exception as e:
            logger.warning("objectives_template_write_failed", error=str(e))
        return backend

    def _resolve(self, virtual_path: str) -> Path:
        """Resolve a virtual path to a local filesystem path."""
        # Normalize: remove leading slash
        rel = virtual_path.lstrip("/")
        # Using joinpath and then resolve to check for traversal
        path = (self.root / rel).resolve()
        if not str(path).startswith(str(self.root.resolve())):
            raise PermissionError(f"Path traversal attempted: {virtual_path}")
        return path

    def _virtual(self, local_path: Path) -> str:
        """Convert a local path back to a virtual path."""
        try:
            rel = local_path.relative_to(self.root)
            return "/" + str(rel)
        except ValueError:
            return "/" + local_path.name

    def sync_to_s3(self) -> None:
        """Push all local files to S3."""
        if not self.s3_backend:
            logger.warning("sync_to_s3_skipped_no_backend")
            return

        s3_root = self.s3_backend._resolve_path("")
        logger.info("sync_to_s3_start", local=str(self.root), s3=s3_root)
        try:
            # s3fs put: put(lpath, rpath, recursive=True)
            # Ensure S3 root ends with / if it doesn't already? s3fs might handle it.
            self.s3_backend._fs.put(str(self.root), s3_root, recursive=True)
            logger.info("sync_to_s3_complete")
        except Exception as e:
            logger.error("sync_to_s3_failed", error=str(e))
            raise

    # --- BackendProtocol Implementation ---

    def ls_info(self, path: str) -> list[ProtocolFileInfo]:
        """List files and directories with metadata."""
        local_path = self._resolve(path)
        if not local_path.exists():
            return []

        results = []
        if local_path.is_dir():
            for entry in local_path.iterdir():
                is_dir = entry.is_dir()
                virt = self._virtual(entry)
                if is_dir and not virt.endswith("/"):
                    virt += "/"

                info: ProtocolFileInfo = {
                    "path": virt,
                    "is_dir": is_dir,
                    "size": entry.stat().st_size if not is_dir else 0,
                    "modified_at": datetime.fromtimestamp(
                        entry.stat().st_mtime
                    ).isoformat(),
                }
                results.append(info)
        else:
            # If path is a file, return its info
            results.append(
                {
                    "path": self._virtual(local_path),
                    "is_dir": False,
                    "size": local_path.stat().st_size,
                    "modified_at": datetime.fromtimestamp(
                        local_path.stat().st_mtime
                    ).isoformat(),
                }
            )

        results.sort(key=lambda x: x["path"])
        return results

    def read(self, file_path: str, offset: int = 0, limit: int = 2000) -> str:
        """Read file content with line numbers."""
        local_path = self._resolve(file_path)
        try:
            content = local_path.read_text(encoding="utf-8")

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

    def write(
        self, file_path: str, content: str, overwrite: bool = False
    ) -> WriteResult:
        """Write content to a new file."""
        local_path = self._resolve(file_path)
        if local_path.exists() and not overwrite:
            return WriteResult(
                error=f"Cannot write to {file_path} because it already exists."
            )

        try:
            local_path.parent.mkdir(parents=True, exist_ok=True)
            local_path.write_text(content, encoding="utf-8")
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
        local_path = self._resolve(file_path)
        if not local_path.exists():
            return EditResult(error=f"Error: File '{file_path}' not found")

        try:
            content = local_path.read_text(encoding="utf-8")

            result = perform_string_replacement(
                content, old_string, new_string, replace_all
            )
            if isinstance(result, str):
                return EditResult(error=result)

            new_content, occurrences = result
            local_path.write_text(new_content, encoding="utf-8")

            return EditResult(
                path=file_path, files_update=None, occurrences=int(occurrences)
            )
        except Exception as e:
            return EditResult(error=f"Error editing file '{file_path}': {e!s}")

    def grep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str:
        """Literal text search in files."""
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
                local_path = self._resolve(f_info["path"])
                content = local_path.read_text(encoding="utf-8")

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
        local_base = self._resolve(path)
        if not local_base.exists() or not local_base.is_dir():
            return []

        results = []
        p = pattern.lstrip("/")

        try:
            if "**" in p:
                matched = local_base.rglob(p.replace("**/", ""))
            else:
                matched = local_base.glob(p)

            for entry in matched:
                is_dir = entry.is_dir()
                virt = self._virtual(entry)
                if is_dir and not virt.endswith("/"):
                    virt += "/"

                results.append(
                    {
                        "path": virt,
                        "is_dir": is_dir,
                        "size": entry.stat().st_size if not is_dir else 0,
                        "modified_at": datetime.fromtimestamp(
                            entry.stat().st_mtime
                        ).isoformat(),
                    }
                )
            results.sort(key=lambda x: x["path"])
            return results
        except Exception as e:
            logger.error("glob_failed", error=str(e))
            return []

    def upload_files(self, files: list[tuple[str, bytes]]) -> list[FileUploadResponse]:
        """Upload multiple files."""
        responses = []
        for path, content in files:
            try:
                local_p = self._resolve(path)
                local_p.parent.mkdir(parents=True, exist_ok=True)
                local_p.write_bytes(content)
                responses.append(FileUploadResponse(path=path, error=None))
            except Exception as e:
                responses.append(FileUploadResponse(path=path, error=str(e)))
        return responses

    def download_files(self, paths: list[str]) -> list[FileDownloadResponse]:
        """Download multiple files."""
        responses = []
        for path in paths:
            try:
                local_p = self._resolve(path)
                content = local_p.read_bytes()
                responses.append(
                    FileDownloadResponse(path=path, content=content, error=None)
                )
            except FileNotFoundError:
                responses.append(
                    FileDownloadResponse(path=path, error="file_not_found")
                )
            except Exception as e:
                responses.append(FileDownloadResponse(path=path, error=str(e)))
        return responses

    # --- Legacy methods ---

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
        try:
            local_path = self._resolve(path)
            return local_path.exists()
        except:
            return False

    def delete(self, path: str) -> None:
        """Delete a file or directory."""
        local_path = self._resolve(path)
        if not local_path.exists():
            raise FileNotFoundError(f"Path not found: {path}")
        if local_path.is_dir():
            shutil.rmtree(local_path)
        else:
            local_path.unlink()
