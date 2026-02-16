import importlib.util
import logging
import sys
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)


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
            exec(script_content, local_scope)
            build_func = local_scope.get("build")
            if not build_func:
                for val in local_scope.values():
                    if callable(val) and getattr(val, "__name__", "") == "build":
                        build_func = val
                        break

            if build_func:
                return build_func()
            raise AttributeError("build() function not found in script content.")
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
        spec = importlib.util.spec_from_file_location(module_name, str(path))
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load spec for {path}")

        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except Exception as e:
            raise RuntimeError(f"Failed to execute script {path}: {e}") from e

        if hasattr(module, "build"):
            return module.build()

        # Fallback for finding a 'build' function in the relative scope
        for attr in dir(module):
            val = getattr(module, attr)
            if callable(val) and attr == "build":
                return val()

        raise AttributeError(f"build() function not found in script {path}.")
