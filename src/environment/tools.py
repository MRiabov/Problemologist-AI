"""
[DEPRECATED] This module is deprecated. Use src.environment.runtime.ToolRuntime instead.
This file is kept for backward compatibility with scripts that might import these functions directly.
"""

import functools
import warnings
from typing import Optional, Any

from src.environment.runtime import ToolRuntime

# Global runtime instance for backward compatibility
# Note: This uses the default "workspace" directory.
_RUNTIME = ToolRuntime("workspaces/main")


def _deprecated(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        # warnings.warn(f"Function {func.__name__} is deprecated. Use ToolRuntime.{func.__name__} instead.", DeprecationWarning, stacklevel=2)
        return func(*args, **kwargs)

    return wrapper


# Expose global variables to avoid breaking imports
WORKSPACE_DIR = _RUNTIME.workspace_dir
_SANDBOX = _RUNTIME.sandbox
_ACTIVE_SESSION_ID = _RUNTIME.active_session_id
_PART_INDEX = _RUNTIME.part_index
_WORKBENCHES = _RUNTIME.workbenches


@_deprecated
def start_session(session_id: str = "agent-session") -> str:
    return _RUNTIME.start_session(session_id)


@_deprecated
def stop_session() -> str:
    return _RUNTIME.stop_session()


@_deprecated
def set_workspace_dir(path: str):
    global WORKSPACE_DIR, _SANDBOX
    _RUNTIME.workspace_dir = path  # This is a bit hacky as Runtime doesn't fully support hot-swap of workspace path perfectly
    # Re-init sandbox
    from src.environment.sandbox import PodmanSandbox
    from pathlib import Path

    _RUNTIME.sandbox = PodmanSandbox(path)
    WORKSPACE_DIR = path
    _SANDBOX = _RUNTIME.sandbox


@_deprecated
def write_file(content: str, path: str, mode: str = "overwrite") -> str:
    return _RUNTIME.write_file(content, path, mode)


@_deprecated
def edit_file(path: str, find: str, replace: str) -> str:
    return _RUNTIME.edit_file(path, find, replace)


@_deprecated
def write_script(content: str, filename: str = "design.py") -> str:
    return write_file(content, filename, mode="overwrite")


@_deprecated
def edit_script(filename: str, find: str, replace: str) -> str:
    return edit_file(filename, find, replace)


@_deprecated
def read_script(filename: str = "design.py") -> str:
    return _RUNTIME.read_script(filename)


@_deprecated
def view_file(path: str) -> str:
    return _RUNTIME.view_file(path)


@_deprecated
def preview_design(filename: str = "design.py") -> str:
    return _RUNTIME.preview_design(filename)


@_deprecated
def read_skill(
    skill_name: str, filename: str = "SKILL.md", resource_type: Optional[str] = None
) -> str:
    return _RUNTIME.read_skill(skill_name, filename, resource_type)


@_deprecated
def run_command(command: str, timeout: int = 60) -> str:
    return _RUNTIME.run_command(command, timeout)


@_deprecated
def run_skill_script(skill_name: str, script_name: str, arguments: str = "") -> str:
    return _RUNTIME.run_skill_script(skill_name, script_name, arguments)


@_deprecated
def list_skills() -> str:
    return _RUNTIME.list_skills()


@_deprecated
def list_skill_files(skill_name: str) -> str:
    return _RUNTIME.list_skill_files(skill_name)


@_deprecated
def update_skill(
    skill_name: str,
    content: str,
    filename: str = "SKILL.md",
    resource_type: Optional[str] = None,
) -> str:
    return _RUNTIME.update_skill(skill_name, content, filename, resource_type)


@_deprecated
def init_skill(skill_name: str) -> str:
    return _RUNTIME.init_skill(skill_name)


@_deprecated
def package_skill(skill_name: str) -> str:
    return _RUNTIME.package_skill(skill_name)


@_deprecated
def _analyze_cached(
    file_hash: str, default_process: str, default_quantity: int, script_content: str
):
    # This was an internal method, mapping to tool runtime logical implementation
    # But check_manufacturability in Runtime now handles this.
    # Exposing specific cache method is hard.
    warnings.warn(
        "_analyze_cached is deprecated and no longer supported directly.",
        DeprecationWarning,
    )
    return {}


@_deprecated
def check_manufacturability(
    design_file: str = "design.py", process: str = "cnc", quantity: int = 1
) -> dict:
    return _RUNTIME.check_manufacturability(design_file, process, quantity)


@_deprecated
def search_docs(query: str) -> str:
    return _RUNTIME.search_docs(query)


@_deprecated
def search_parts(query: str) -> str:
    return _RUNTIME.search_parts(query)


@_deprecated
def preview_part(part_id: str) -> str:
    return _RUNTIME.preview_part(part_id)


@_deprecated
def lint_script(filename: str = "design.py") -> str:
    return _RUNTIME.lint_script(filename)
