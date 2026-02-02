from pathlib import Path
from typing import Any

import yaml

# Cache for loaded prompts
_PROMPTS_CACHE: dict[str, Any] = {}


def load_prompts() -> dict[str, Any]:
    """
    Loads prompts from the prompts.yaml file and caches them.
    """
    global _PROMPTS_CACHE
    if _PROMPTS_CACHE:
        return _PROMPTS_CACHE

    # Locate the prompts.yaml relative to this file
    base_dir = Path(__file__).resolve().parent.parent
    prompts_path = base_dir / "prompts.yaml"

    if not prompts_path.exists():
        # Fallback or error
        raise FileNotFoundError(f"Prompts file not found at {prompts_path}")

    with prompts_path.open() as f:
        _PROMPTS_CACHE = yaml.safe_load(f)

    return _PROMPTS_CACHE


def get_prompt(path: str, default: str = "") -> str:
    """
    Retrieves a prompt by a dot-separated path (e.g., 'planner.system').

    Args:
        path: Dot-separated path to the prompt.
        default: Default value if prompt is not found.
    """
    prompts = load_prompts()
    keys = path.split(".")

    value = prompts
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            return default

    return value if isinstance(value, str) else default
