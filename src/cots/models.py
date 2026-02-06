from typing import Any, Dict, Literal, Optional
from pydantic import BaseModel


class COTSItem(BaseModel):
    part_id: str
    name: str
    category: Literal["fastener", "motor", "gear", "bearing", "electronic"]
    unit_cost: float
    weight_g: float
    import_recipe: str  # build123d code snippet
    metadata: Dict[str, Any]


class SearchQuery(BaseModel):
    query: str
    constraints: Optional[Dict[str, Any]] = None  # e.g., {"max_weight": 100}
    limit: int = 5
