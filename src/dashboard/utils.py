from pathlib import Path


def get_project_root() -> Path:
    """Returns the absolute path to the project root."""
    return Path(__file__).parent.parent.parent.absolute()


def resolve_artifact_path(path_or_name: str) -> Path:
    """
    Resolves the path to an artifact.
    First checks if it's a relative path from project root,
    then checks in the default workspace/artifacts folder.
    """
    root = get_project_root()

    # Try as direct relative path (from DB)
    direct_path = root / path_or_name
    if direct_path.exists():
        return direct_path

    # Try in default artifacts folder
    fallback_path = root / "workspace" / "artifacts" / path_or_name
    return fallback_path
