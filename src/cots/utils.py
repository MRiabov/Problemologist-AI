import json
import os
from typing import Dict, Any, Optional

# Global cache for descriptions
_DESCRIPTIONS: Optional[Dict[str, Any]] = None


def load_descriptions() -> Dict[str, Any]:
    """Load COTS descriptions from the assets directory."""
    global _DESCRIPTIONS
    if _DESCRIPTIONS is not None:
        return _DESCRIPTIONS

    # Locate the assets directory relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # Assuming src/cots/utils.py, assets should be at src/assets/cots_descriptions.json
    assets_path = os.path.join(current_dir, "..", "assets", "cots_descriptions.json")

    if not os.path.exists(assets_path):
        # Fallback if the above path logic fails for some reason
        # (e.g. if we are running from a different worktree structure)
        assets_path = os.path.abspath(
            os.path.join(os.getcwd(), "src", "assets", "cots_descriptions.json")
        )

    try:
        with open(assets_path, "r") as f:
            _DESCRIPTIONS = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Warning: Could not load COTS descriptions from {assets_path}: {e}")
        _DESCRIPTIONS = {}

    return _DESCRIPTIONS


def get_description(part_id: str) -> Dict[str, Any]:
    """
    Get description and metadata for a part ID.
    Supports exact match and partial match (after the semicolon or colon).
    """
    data = load_descriptions()

    # Try exact match
    if part_id in data:
        return data[part_id]

    # Try partial match (e.g. "bd_warehouse:motor:Nema17" -> "Nema17")
    # Take the last part after the last colon
    short_id = part_id.split(":")[-1]

    if short_id in data:
        return data[short_id]

    return {"description": "No description available for this part.", "metadata": {}}
