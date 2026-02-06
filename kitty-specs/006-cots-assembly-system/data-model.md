# Data Model: COTS Assembly System

## Core Models (Pydantic)

### 1. COTSPartSummary

```python
class COTSPartSummary(BaseModel):
    id: str
    name: str
    category: str
    provider: str = "bd_warehouse"
```

### 2. COTSPartDetail

```python
class COTSPartDetail(BaseModel):
    id: str
    name: str
    image_url: str  # S3 URL
    description: str
    recipe: str      # Python instantiation code
    properties: Dict[str, Any] # mass, dimensions, etc.
```

## Persistence

1. **Catalog**: Stored as a versioned JSON/SQLite file on **S3**.
2. **Search Index**: Maintained by the **Catalog Sub-Agent** in memory or via a lightweight vector store if the catalog grows.
3. **Observation DB**: Selected part IDs are recorded in the global trace for reproducibility.
