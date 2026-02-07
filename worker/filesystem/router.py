"""Filesystem router with read-only restrictions.

Provides a unified filesystem view that overlays S3-backed workspace
with local read-only directories (utils/, skills/, reviews/).
"""

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import structlog

from .backend import FileInfo, SandboxFilesystemBackend

logger = structlog.get_logger(__name__)


class AccessMode(Enum):
    """Access modes for filesystem paths."""

    READ_WRITE = "read_write"
    READ_ONLY = "read_only"


@dataclass
class MountPoint:
    """A mounted directory in the virtual filesystem."""

    virtual_prefix: str  # e.g., "/utils"
    local_path: Path  # e.g., /app/worker/utils
    access_mode: AccessMode


class WritePermissionError(Exception):
    """Raised when a write operation is attempted on a read-only path."""

    pass


@dataclass
class FilesystemRouter:
    """Routes filesystem operations to appropriate backends.

    Provides a unified view of:
    - files/ (workspace) -> S3 (Read/Write)
    - utils/, skills/, reviews/ -> Local container disk (Read Only)
    """

    s3_backend: SandboxFilesystemBackend
    mount_points: list[MountPoint] = field(default_factory=list)

    # Paths that are always read-only
    READ_ONLY_PREFIXES: tuple[str, ...] = ("/utils/", "/skills/", "/reviews/")

    def __post_init__(self) -> None:
        """Initialize default mount points if none provided."""
        if not self.mount_points:
            # Set up default read-only mounts
            # These point to the worker's local directories
            base = Path(__file__).parent.parent
            self.mount_points = [
                MountPoint(
                    virtual_prefix="/utils",
                    local_path=base / "utils",
                    access_mode=AccessMode.READ_ONLY,
                ),
                MountPoint(
                    virtual_prefix="/skills",
                    local_path=Path("/app/skills"),
                    access_mode=AccessMode.READ_ONLY,
                ),
                MountPoint(
                    virtual_prefix="/reviews",
                    local_path=base / "reviews",
                    access_mode=AccessMode.READ_ONLY,
                ),
            ]

    def _is_read_only(self, path: str) -> bool:
        """Check if a path is in a read-only zone.

        Args:
            path: Virtual path to check.

        Returns:
            True if path is read-only, False otherwise.
        """
        normalized = path if path.startswith("/") else f"/{path}"
        return any(normalized.startswith(prefix) for prefix in self.READ_ONLY_PREFIXES)

    def _get_mount_point(self, path: str) -> MountPoint | None:
        """Find the mount point for a virtual path.

        Args:
            path: Virtual path.

        Returns:
            MountPoint if path matches a mounted directory, None otherwise.
        """
        normalized = path if path.startswith("/") else f"/{path}"
        for mount in self.mount_points:
            if normalized.startswith(mount.virtual_prefix):
                return mount
        return None

    def _resolve_local_path(self, path: str, mount: MountPoint) -> Path:
        """Resolve a virtual path to a local filesystem path.

        Args:
            path: Virtual path.
            mount: Mount point containing this path.

        Returns:
            Resolved local Path.
        """
        normalized = path if path.startswith("/") else f"/{path}"
        relative = normalized[len(mount.virtual_prefix) :].lstrip("/")
        return mount.local_path / relative

    def ls(self, path: str = "/") -> list[FileInfo]:
        """List contents of a directory.

        For root ("/"), merges S3 contents with mounted directories.

        Args:
            path: Virtual directory path.

        Returns:
            List of FileInfo objects for directory contents.
        """
        logger.debug("router_ls", path=path)

        normalized = path if path.startswith("/") else f"/{path}"

        # Check if path is in a mounted directory
        mount = self._get_mount_point(normalized)
        if mount:
            return self._ls_local(normalized, mount)

        # For root or S3 paths, get S3 contents and merge with mounts
        s3_contents = self.s3_backend.ls(path)

        # If at root, add mount points that exist
        if normalized == "/" or normalized == "":
            for mount in self.mount_points:
                if mount.local_path.exists():
                    name = mount.virtual_prefix.strip("/")
                    s3_contents.append(
                        FileInfo(
                            path=mount.virtual_prefix,
                            name=name,
                            is_dir=True,
                            size=None,
                        )
                    )

        return s3_contents

    def _ls_local(self, path: str, mount: MountPoint) -> list[FileInfo]:
        """List contents of a local mounted directory.

        Args:
            path: Virtual path.
            mount: Mount point for this path.

        Returns:
            List of FileInfo objects.
        """
        local_path = self._resolve_local_path(path, mount)

        if not local_path.exists():
            return []

        result = []
        for entry in local_path.iterdir():
            virtual = f"{mount.virtual_prefix}/{entry.relative_to(mount.local_path)}"
            result.append(
                FileInfo(
                    path=virtual,
                    name=entry.name,
                    is_dir=entry.is_dir(),
                    size=entry.stat().st_size if entry.is_file() else None,
                )
            )

        return result

    def read(self, path: str) -> bytes:
        """Read file contents.

        Args:
            path: Virtual file path.

        Returns:
            File contents as bytes.

        Raises:
            FileNotFoundError: If file does not exist.
        """
        logger.debug("router_read", path=path)

        normalized = path if path.startswith("/") else f"/{path}"

        # Check if in a mounted directory
        mount = self._get_mount_point(normalized)
        if mount:
            local_path = self._resolve_local_path(normalized, mount)
            if not local_path.exists():
                raise FileNotFoundError(f"File not found: {path}")
            return local_path.read_bytes()

        # Otherwise read from S3
        return self.s3_backend.read(path)

    def write(self, path: str, content: bytes | str) -> None:
        """Write content to a file.

        Args:
            path: Virtual file path.
            content: Content to write.

        Raises:
            PermissionError: If path is in a read-only directory.
        """
        logger.debug("router_write", path=path)

        if self._is_read_only(path):
            logger.warning("router_write_blocked", path=path)
            raise WritePermissionError(f"Cannot write to read-only path: {path}")

        self.s3_backend.write(path, content)

    def edit(self, path: str, old_content: str, new_content: str) -> bool:
        """Edit file by replacing content.

        Args:
            path: Virtual file path.
            old_content: Content to find and replace.
            new_content: Replacement content.

        Returns:
            True if replacement was made, False if old_content not found.

        Raises:
            PermissionError: If path is in a read-only directory.
            FileNotFoundError: If file does not exist.
        """
        logger.debug("router_edit", path=path)

        if self._is_read_only(path):
            logger.warning("router_edit_blocked", path=path)
            raise WritePermissionError(f"Cannot edit read-only path: {path}")

        return self.s3_backend.edit(path, old_content, new_content)

    def exists(self, path: str) -> bool:
        """Check if a path exists.

        Args:
            path: Virtual path to check.

        Returns:
            True if path exists, False otherwise.
        """
        normalized = path if path.startswith("/") else f"/{path}"

        # Check mounted directories
        mount = self._get_mount_point(normalized)
        if mount:
            local_path = self._resolve_local_path(normalized, mount)
            return local_path.exists()

        # Check S3
        return self.s3_backend.exists(path)

    def delete(self, path: str) -> None:
        """Delete a file or directory.

        Args:
            path: Virtual path to delete.

        Raises:
            PermissionError: If path is in a read-only directory.
            FileNotFoundError: If path does not exist.
        """
        logger.debug("router_delete", path=path)

        if self._is_read_only(path):
            logger.warning("router_delete_blocked", path=path)
            raise WritePermissionError(f"Cannot delete read-only path: {path}")

        self.s3_backend.delete(path)


def create_filesystem_router(
    session_id: str,
    custom_mounts: list[MountPoint] | None = None,
) -> FilesystemRouter:
    """Create a filesystem router with standard configuration.

    Args:
        session_id: Session ID for S3 path isolation.
        custom_mounts: Optional custom mount points.

    Returns:
        Configured FilesystemRouter instance.
    """
    from .backend import SandboxFilesystemBackend

    s3_backend = SandboxFilesystemBackend.create(session_id)

    if custom_mounts:
        return FilesystemRouter(s3_backend=s3_backend, mount_points=custom_mounts)

    return FilesystemRouter(s3_backend=s3_backend)
