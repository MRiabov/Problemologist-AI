# Quickstart: COTS Assembly System

## 1. Agent Usage (In Reasoning Loop)

The agent interacts with the COTS catalog through the **Catalog Sub-Agent**.

```text
ENGINEER: "I need an M6 hex nut for the assembly."
SUB-AGENT: Invoke search_parts(query="M6 hex nut")
SUB-AGENT: Return [COTSPartSummary(id="bdw:fastener:HexNut:M6", ...)]
ENGINEER: "Preview the M6 nut."
SUB-AGENT: Invoke preview_part(id="bdw:fastener:HexNut:M6")
SUB-AGENT: Return COTSPartDetail(recipe="HexNut('M6')", ...)
```

## 2. API Integration

Trigger a catalog search from the Controller:

```python
import requests

response = requests.get(
    "http://controller:8000/api/catalog/search",
    params={"q": "NEMA 17"}
)
```

## 3. Maintenance

To update the catalog index after adding new parts to `bd_warehouse`:

```bash
python -m src.catalog.builder --rebuild
```
