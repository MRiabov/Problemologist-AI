from enum import Enum
from typing import Any, Dict, Optional
from pydantic import BaseModel, StrictStr, StrictFloat, StrictInt


class COTSCategory(str, Enum):
    FASTENER = "fastener"
    MOTOR = "motor"
    GEAR = "gear"
    BEARING = "bearing"
    ELECTRONIC = "electronic"


class COTSItem(BaseModel):
    part_id: StrictStr
    name: StrictStr
    category: COTSCategory
    unit_cost: StrictFloat
    weight_g: StrictFloat
    import_recipe: StrictStr  # build123d code snippet
    metadata: Dict[StrictStr, Any]


class SearchQuery(BaseModel):
    query: StrictStr
    constraints: Optional[Dict[StrictStr, Any]] = None  # e.g., {"max_weight": 100}
    limit: StrictInt = 5
