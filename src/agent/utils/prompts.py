import os
import yaml
from typing import Dict, Any

# Cache for loaded prompts
_PROMPTS_CACHE: Dict[str, Any] = {}


def load_prompts() -> Dict[str, Any]:
    """
    Loads prompts from the prompts.yaml file and caches them.
    """
    global _PROMPTS_CACHE
    if _PROMPTS_CACHE:
        return _PROMPTS_CACHE

    # Locate the prompts.yaml relative to this file
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    prompts_path = os.path.join(base_dir, "prompts.yaml")

    if not os.path.exists(prompts_path):
        # Fallback or error
        raise FileNotFoundError(f"Prompts file not found at {prompts_path}")

    with open(prompts_path, "r") as f:
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
