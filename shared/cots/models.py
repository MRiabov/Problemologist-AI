from enum import Enum
from typing import Any

from pydantic import BaseModel, StrictFloat, StrictInt, StrictStr


class COTSCategory(str, Enum):
    FASTENER = "fastener"
    MOTOR = "motor"
    GEAR = "gear"
    BEARING = "bearing"
    ELECTRONIC = "electronic"
    POWER_SUPPLY = "power_supply"
    RELAY = "relay"
    CONNECTOR = "connector"
    WIRE = "wire"


class COTSItem(BaseModel):
    part_id: StrictStr
    name: StrictStr
    category: COTSCategory
    unit_cost: StrictFloat
    weight_g: StrictFloat
    import_recipe: StrictStr  # build123d code snippet
    metadata: dict[StrictStr, Any]


class SearchQuery(BaseModel):
    query: StrictStr
    constraints: dict[StrictStr, Any] | None = None  # e.g., {"max_weight": 100}
    limit: StrictInt = 5
