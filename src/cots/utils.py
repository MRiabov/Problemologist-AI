import json
from pathlib import Path
from typing import Any

from src.agent.utils.logging import get_logger

logger = get_logger(__name__)

# Global cache for descriptions
_DESCRIPTIONS: dict[str, Any] | None = None


def load_descriptions() -> dict[str, Any]:
    """Load COTS descriptions from the assets directory."""
    global _DESCRIPTIONS
    if _DESCRIPTIONS is not None:
        return _DESCRIPTIONS

    # Locate the assets directory relative to this file
    current_dir = Path(__file__).resolve().parent
    # Assuming src/cots/utils.py, assets should be at src/assets/cots_descriptions.json
    assets_path = current_dir.parent / "assets" / "cots_descriptions.json"

    if not assets_path.exists():
        # Fallback if the above path logic fails for some reason
        # (e.g. if we are running from a different worktree structure)
        assets_path = Path.cwd() / "src" / "assets" / "cots_descriptions.json"

    try:
        with assets_path.open() as f:
            _DESCRIPTIONS = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        logger.warning(
            "Could not load COTS descriptions", path=str(assets_path), error=str(e)
        )
        _DESCRIPTIONS = {}

    return _DESCRIPTIONS


def get_description(part_id: str) -> dict[str, Any]:
    """
    Get description and metadata for a part ID.
    Supports exact match, partial match, and keyword-based fallback.
    """
    data = load_descriptions()

    # 1. Try exact match
    if part_id in data:
        return data[part_id]

    # 2. Try partial match (e.g. "bd_warehouse:motor:Nema17" -> "Nema17")
    # Take the last part after the last colon
    short_id = part_id.split(":")[-1]
    if short_id in data:
        return data[short_id]

    # 3. Try keyword match for common types (motor, bearing, screw, nut)
    for keyword in ["motor", "bearing", "screw", "nut"]:
        if keyword in part_id.lower():
            # Find the first entry in data that contains the keyword
            for key, val in data.items():
                if (
                    keyword in key.lower()
                    or keyword in val.get("metadata", {}).get("type", "").lower()
                ):
                    return val

    # Default fallback
    return {"description": "No description available for this part.", "metadata": {}}
