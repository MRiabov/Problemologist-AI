"""Filesystem utilities for the worker environment."""

import os

from deepagents.middlewares import FilesystemMiddleware

from ..filesystem.backend import LocalFilesystemBackend


def get_filesystem_middleware(session_id: str = None):
    """
    Configure FilesystemMiddleware with LocalFilesystemBackend.
    Allows agents to interact with the sandboxed filesystem using standard tools.
    """
    if session_id is None:
        session_id = os.getenv("SESSION_ID", "default")

    backend = LocalFilesystemBackend.create(session_id)
    # Note: FilesystemMiddleware from deepagents provides ls, read, write tools
    return FilesystemMiddleware(backend=backend)
