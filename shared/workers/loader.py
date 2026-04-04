import importlib.util
import logging
import os
import sys
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from build123d import Compound, Part

logger = logging.getLogger(__name__)
SCRIPT_IMPORT_MODE_ENV = "PROBLEMOLOGIST_SCRIPT_IMPORT_MODE"


@contextmanager
def sys_path_context(path: str):
    """Context manager to temporarily add a path to sys.path."""
    from contextlib import suppress

    if path not in sys.path:
        sys.path.insert(0, path)
        try:
            yield
        finally:
            with suppress(ValueError):
                sys.path.remove(path)
    else:
        yield


@contextmanager
def script_import_mode():
    previous = os.environ.get(SCRIPT_IMPORT_MODE_ENV)
    os.environ[SCRIPT_IMPORT_MODE_ENV] = "1"
    try:
        yield
    finally:
        if previous is None:
            os.environ.pop(SCRIPT_IMPORT_MODE_ENV, None)
        else:
            os.environ[SCRIPT_IMPORT_MODE_ENV] = previous


@contextmanager
def working_directory(path: str | Path):
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def _is_loaded_component_candidate(value: Any) -> bool:
    return isinstance(value, (Compound, Part))


def _extract_loaded_component(namespace: dict[str, Any], origin: str) -> Any:
    build_func = namespace.get("build")
    if callable(build_func):
        return build_func()

    for name in ("result", "compound", "benchmark", "assembly", "solution", "part"):
        value = namespace.get(name)
        if _is_loaded_component_candidate(value):
            return value

    candidates = [
        (name, value)
        for name, value in namespace.items()
        if not name.startswith("__") and _is_loaded_component_candidate(value)
    ]
    if len(candidates) == 1:
        return candidates[0][1]
    if len(candidates) > 1:
        candidate_names = ", ".join(name for name, _ in candidates[:5])
        raise AttributeError(
            "Multiple top-level build123d objects found in "
            f"{origin}; expose the final assembly as `result = ...` "
            f"or define build(). Candidates: {candidate_names}"
        )

    raise AttributeError(
        f"No build123d result found in {origin}. Define build() or bind the "
        "final assembly to a top-level variable such as `result`."
    )


def load_component_from_script(
    script_path: str | Path,
    session_root: str | Path | None = None,
    script_content: str | None = None,
) -> Any:
    """
    Loads the component from the specified build123d script.

    Args:
        script_path: Path to the .py script.
        session_root: Root directory for local imports (defaults to script's parent).
        script_content: Optional direct script content (deprecated).

    Returns:
        The result of the build() function.
    """
    if script_content:
        logger.warning("load_component_using_script_content_deprecated")
        local_scope = {}
        try:
            with script_import_mode():
                exec(script_content, local_scope)
            return _extract_loaded_component(local_scope, "script content")
        except Exception as e:
            raise RuntimeError(f"Failed to execute script content: {e}") from e

    path = Path(script_path)
    if not path.exists():
        raise FileNotFoundError(f"Script not found at {path.absolute()}")

    # Determine session root for local imports
    root = str(session_root or path.parent)

    # Use a unique module name to prevent sys.modules collisions (Review Item 2)
    module_name = f"dynamic_build_{uuid.uuid4().hex}"

    with sys_path_context(root):
        with working_directory(root):
            spec = importlib.util.spec_from_file_location(module_name, str(path))
            if spec is None or spec.loader is None:
                raise RuntimeError(f"Could not load spec for {path}")

            module = importlib.util.module_from_spec(spec)
            try:
                with script_import_mode():
                    spec.loader.exec_module(module)
            except Exception as e:
                raise RuntimeError(f"Failed to execute script {path}: {e}") from e

            try:
                return _extract_loaded_component(vars(module), str(path))
            finally:
                # Prevent memory leak by removing unique module from sys.modules if it was added
                if module_name in sys.modules:
                    sys.modules.pop(module_name)
