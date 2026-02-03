import json
from typing import Any

from src.utils.paths import get_assets_dir

# Global cache for descriptions
_DESCRIPTIONS: dict[str, Any] | None = None


def load_descriptions() -> dict[str, Any]:
    """Load COTS descriptions from the assets directory."""
    global _DESCRIPTIONS
    if _DESCRIPTIONS is not None:
        return _DESCRIPTIONS

    # Use standard assets directory
    assets_path = get_assets_dir() / "cots_descriptions.json"

    try:
        with assets_path.open() as f:
            _DESCRIPTIONS = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Warning: Could not load COTS descriptions from {assets_path}: {e}")
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
