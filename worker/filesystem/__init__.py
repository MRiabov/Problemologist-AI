"""Worker filesystem package.

Provides S3-backed sandboxed filesystem with session isolation
and read-only mount points for utils, skills, and reviews.
"""

from .backend import (
    FileInfo,
    SandboxFilesystemBackend,
    SessionManager,
    SimpleSessionManager,
)
from .db import (
    S3Config,
    get_s3_filesystem,
    get_verified_s3_filesystem,
    verify_s3_connection,
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
    "MountPoint",
    "S3Config",
    "SandboxFilesystemBackend",
    "SessionManager",
    "SimpleSessionManager",
    "WritePermissionError",
    "create_filesystem_router",
    "get_s3_filesystem",
    "get_verified_s3_filesystem",
    "verify_s3_connection",
]
