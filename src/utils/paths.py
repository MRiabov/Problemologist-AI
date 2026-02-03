from pathlib import Path


def get_project_root() -> Path:
    """Returns the absolute path to the project root directory."""
    # This file is in src/utils/paths.py
    return Path(__file__).resolve().parent.parent.parent


def get_workspace_dir() -> Path:
    """Returns the absolute path to the default workspace directory."""
    return get_project_root() / "workspace"


def get_assets_dir() -> Path:
    """Returns the absolute path to the assets directory."""
    return get_project_root() / "src" / "assets"


def get_config_dir() -> Path:
    """Returns the absolute path to the config directory."""
    return get_project_root() / "config"


def get_skills_dir() -> Path:
    """Returns the absolute path to the skills directory."""
    return get_project_root() / "docs" / "skills"
