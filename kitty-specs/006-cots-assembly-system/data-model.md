# Data Model: COTS Assembly System

**Feature**: 006-cots-assembly-system

## Entities

### PartSummary
Lightweight object returned by search.

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | `str` | Unique identifier (e.g., `bd_warehouse:motor:Nema17`) |
| `name` | `str` | Human/Agent readable name (e.g., "NEMA 17 Stepper Motor") |
| `provider` | `str` | Source identifier (e.g., `bd_warehouse`) |

### PartPreview
Rich object returned by preview tool.

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | `str` | Unique identifier |
| `image_path` | `str` | Absolute path to the rendered image (PNG) |
| `description` | `str` | Natural language text describing features/orientation |
| `metadata` | `dict` | Physical properties (mass, dimensions, voltage, etc.) |
| `recipe` | `str` | Python code snippet to instantiate this part |

### Part (Internal)
Internal representation within the Index.

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | `str` | Unique identifier |
| `factory` | `Callable` | Function to create the `build123d` object |
| `params` | `dict` | Parameters to pass to the factory |
