from functools import lru_cache
from pathlib import Path

import yaml

from shared.logging import get_logger

logger = get_logger(__name__)

# Adjust path based on where this file is located relative to project root
CONFIG_PATH = Path(__file__).parent.parent / "config" / "prompts.yaml"


@lru_cache
def load_prompts():
    if not CONFIG_PATH.exists():
        logger.error(
            "prompts_config_not_found", path=str(CONFIG_PATH), session_id="system"
        )
        raise FileNotFoundError(f"Prompts config not found at {CONFIG_PATH}")
    with CONFIG_PATH.open("r") as f:
        data = yaml.safe_load(f)
        if not data:
            logger.error(
                "prompts_config_empty", path=str(CONFIG_PATH), session_id="system"
            )
            raise ValueError(f"Prompts config is empty at {CONFIG_PATH}")
        return data

