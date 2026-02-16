"""Worker filesystem package.

Provides S3-backed sandboxed filesystem with session isolation
and read-only mount points for utils, skills, and reviews.
"""

from .backend import (
    FileInfo,
    LocalFilesystemBackend,
    SandboxFilesystemBackend,
    SessionManager,
    SimpleSessionManager,
)
from .router import (
    AccessMode,
    FilesystemRouter,
    MountPoint,
    WritePermissionError,
    create_filesystem_router,
)

__all__ = [
    "AccessMode",
    "FileInfo",
    "FilesystemRouter",
    "LocalFilesystemBackend",
    "SandboxFilesystemBackend",
    "MountPoint",
    "SessionManager",
    "SimpleSessionManager",
    "WritePermissionError",
    "create_filesystem_router",
]
