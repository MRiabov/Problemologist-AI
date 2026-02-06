# Research: COTS Assembly System

**Feature**: 006-cots-assembly-system
**Created**: 2026-02-01

## 1. Technical Decisions

### 1.1. Part Description Storage

* **Decision**: JSON-based descriptions (`cots_descriptions.json`) stored on S3 and loaded by the **Catalog Sub-Agent**.
* **Rationale**:
  * **Performance**: Sub-agent executes specialized SQL/Grep queries off-loop.
  * **Scalability**: S3 storage allows all workers to access the same catalog without bundling large assets.

### 1.2. Part Import Instructions

* **Decision**: Recipes are Python snippets injected into the worker sandbox via tools.
* **Rationale**: Direct instantiation via `bd_warehouse` in the worker environment.

### 1.3. Search Pattern

* **Decision**: Use a **Sub-Agent** for catalog interaction.
* **Rationale**: As per the `desired_architecture.md`, the planner or engineer can prompt a searching sub-agent. This encapsulates the database complexity and provides a natural language interface for part selection.

## 2. Dependencies

* **Worker Runtime**: `bd_warehouse`.
* **Controller**: `pydantic`.

## 3. Integration Patterns

### 3.1. `PartProvider` Interface

```python
class PartProvider(ABC):
    @abstractmethod
    def search(self, query: str) -> List[PartSummary]: ...
    
    @abstractmethod
    def get_part(self, part_id: str) -> Part: ...
```

### 3.2. `BDWarehouseProvider`

* Will use `bd_warehouse` class methods (e.g., `HexNut.sizes()`) to iterate available parts and populate the index.
* Will map `part_id` (e.g., `bd_warehouse:fastener:HexNut:M6-1`) to the specific instantiation call.
