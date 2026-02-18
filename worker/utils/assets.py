import ast
from pathlib import Path

import structlog
from fastapi import HTTPException

logger = structlog.get_logger(__name__)


def validate_asset_source(session_root: Path, asset_path: str) -> None:
    """
    Checks if requesting a model asset (.glb, .stl) has valid source code.
    Raises HTTPException if syntax errors are found in the likely source file.
    """
    if not (asset_path.endswith(".glb") or asset_path.endswith(".stl")):
        return

    # Heuristic: find the source python file
    candidate_paths = [
        Path(asset_path).with_suffix(".py").name,
        "main.py",
        "component.py",
        "solution.py",
    ]

    for py_name in candidate_paths:
        py_path = session_root / py_name
        if py_path.exists():
            try:
                source_code = py_path.read_text(encoding="utf-8")
                ast.parse(source_code)
                return  # Found valid source
            except SyntaxError:
                logger.warning(
                    "asset_serving_refused_syntax_error",
                    asset=asset_path,
                    source=str(py_path),
                )
                raise HTTPException(
                    status_code=422,
                    detail=f"Source code {py_name} has syntax errors.",
                )
            except Exception as e:
                logger.warning("asset_source_check_failed", error=str(e))


def get_media_type(path: str) -> str:
    """Returns the media type for a given file path."""
    if path.endswith(".glb"):
        return "model/gltf-binary"
    if path.endswith(".py"):
        return "text/x-python"
    if path.endswith(".stl"):
        return "model/stl"
    if path.endswith(".png"):
        return "image/png"
    if path.endswith(".jpg") or path.endswith(".jpeg"):
        return "image/jpeg"
    if path.endswith(".mp4"):
        return "video/mp4"
    return "application/octet-stream"
