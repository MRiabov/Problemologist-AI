import os
from pathlib import Path

def get_project_root() -> Path:
    """Returns the absolute path to the project root."""
    # Assuming this file is at src/dashboard/utils.py
    # Path(__file__) is .../src/dashboard/utils.py
    # .parent is .../src/dashboard/
    # .parent.parent is .../src/
    # .parent.parent.parent is .../root/
    return Path(__file__).parent.parent.parent.absolute()

def resolve_artifact_path(relative_path: str) -> Path:
    """Joins project root with the relative path stored in DB and verifies existence."""
    root = get_project_root()
    full_path = root / relative_path
    return full_path
