from pathlib import Path
from functools import lru_cache
import yaml
from shared.logging import get_logger

logger = get_logger(__name__)

# Adjust path based on where this file is located relative to project root
CONFIG_PATH = Path(__file__).parent.parent / "config" / "prompts.yaml"


@lru_cache
def load_prompts():
    if not CONFIG_PATH.exists():
        logger.error("prompts_config_not_found", path=str(CONFIG_PATH))
        raise FileNotFoundError(f"Prompts config not found at {CONFIG_PATH}")
    with CONFIG_PATH.open("r") as f:
        data = yaml.safe_load(f)
        if not data:
            logger.error("prompts_config_empty", path=str(CONFIG_PATH))
            raise ValueError(f"Prompts config is empty at {CONFIG_PATH}")
        return data


def get_prompt(key: str) -> str:
    """
    Retrieves a prompt template by dot-separated key.
    Example: get_prompt("cad_agent.planner.system")
    """
    data = load_prompts()
    keys = key.split(".")
    value = data
    for k in keys:
        if isinstance(value, dict) and k in value:
            value = value[k]
        else:
            logger.error("prompt_key_not_found", key=key, missing_segment=k)
            raise KeyError(f"Prompt key '{key}' not found in config.")

    if not isinstance(value, str):
        logger.error("prompt_key_not_string", key=key, type=type(value))
        raise ValueError(f"Prompt key '{key}' does not point to a string.")

    return value.strip()
