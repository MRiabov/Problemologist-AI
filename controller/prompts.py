import os
import yaml
from pathlib import Path
from functools import lru_cache

# Adjust path based on where this file is located relative to project root
# Using resolve() to get absolute path
CONFIG_PATH = (Path(__file__).parent.parent / "config" / "prompts.yaml").resolve()


@lru_cache
def load_prompts():
    if not CONFIG_PATH.exists():
        raise FileNotFoundError(f"Prompts config not found at {CONFIG_PATH}")
    with open(CONFIG_PATH, "r") as f:
        data = yaml.safe_load(f)
        if not data:
            raise ValueError(f"Prompts config at {CONFIG_PATH} is empty")
        return data


def get_prompt(key: str) -> str:
    """
    Retrieves a prompt template by dot-separated key.
    Example: get_prompt("benchmark_generator.planner")
    """
    data = load_prompts()
    keys = key.split(".")
    value = data
    for k in keys:
        if isinstance(value, dict) and k in value:
            value = value[k]
        else:
            raise KeyError(f"Prompt key '{key}' not found in config.")

    if not isinstance(value, str):
        raise ValueError(f"Prompt key '{key}' does not point to a string.")

    return value.strip()
