# Quickstart: COTS Assembly System

## Prerequisites

1. Install dependencies:
   ```bash
   uv add bd_warehouse
   ```

## Usage

### 1. Initialize the Index
```python
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider

index = PartIndex()
index.register_provider(BDWarehouseProvider())
```

### 2. Search for Parts
```python
results = index.search("NEMA 17")
for res in results:
    print(f"Found: {res.name} ({res.id})")
```

### 3. Preview a Part
```python
part_id = results[0].id
preview = index.preview(part_id)

print(f"Description: {preview.description}")
print(f"Image: {preview.image_path}")
print(f"Recipe:\n{preview.recipe}")
```

### 4. Use in Agent Tool
The system exposes two tools to the agent:
* `search_parts(query="bearing")`
* `preview_part(part_id="...")`

```