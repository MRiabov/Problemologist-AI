from enum import StrEnum
from typing import Any

from pydantic import BaseModel, StrictFloat, StrictInt, StrictStr


class COTSCategory(StrEnum):
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


class SearchConstraints(BaseModel):
    max_weight_g: float | None = None
    max_cost: float | None = None
    category: str | None = None
    min_size: float | None = None


class SearchQuery(BaseModel):
    query: StrictStr
    constraints: SearchConstraints | None = None
    limit: StrictInt = 5
