# Data Model: COTS Assembly System

## Core Entities (Pydantic Models)

### 1. COTSItem

The structured definition of a catalog part.

```python
class COTSItem(BaseModel):
    part_id: str
    name: str
    category: Literal["fastener", "motor", "gear", "bearing", "electronic"]
    unit_cost: float
    weight_g: float
    import_recipe: str  # build123d code snippet
    metadata: Dict[str, Any]
```

### 2. SearchQuery

Payload for the Catalog Subagent.

```python
class SearchQuery(BaseModel):
    query: str
    constraints: Optional[Dict[str, Any]] = None  # e.g., {"max_weight": 100}
    limit: int = 5
```

## Persistence

1. **Global Catalog**: Stored on S3 as a read-only SQLite/JSON index.
2. **Episode Trace**: Every COTS part used is recorded in the `PartsList` of the `EpisodeMetadata` in the Observability DB.
3. **Cache**: Common search results are cached on the Controller to reduce subagent runs.
