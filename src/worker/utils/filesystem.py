"""Filesystem utilities for the worker environment."""

import os
from deepagents.middlewares import FilesystemMiddleware
from ..filesystem.backend import SandboxFilesystemBackend

def get_filesystem_middleware(session_id: str = None):
    """
    Configure FilesystemMiddleware with SandboxFilesystemBackend.
    Allows agents to interact with the sandboxed filesystem using standard tools.
    """
    if session_id is None:
        session_id = os.getenv("SESSION_ID", "default")
        
    backend = SandboxFilesystemBackend.create(session_id)
    # Note: FilesystemMiddleware from deepagents provides ls, read, write tools
    return FilesystemMiddleware(backend=backend)
