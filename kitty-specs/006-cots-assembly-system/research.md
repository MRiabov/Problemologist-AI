# Research: COTS Assembly System

**Feature**: 006-cots-assembly-system
**Created**: 2026-02-01

## 1. Technical Decisions

### 1.1. Part Description Storage
* **Decision**: Pre-generate descriptions and store in a JSON file (`cots_descriptions.json`), loaded into memory at runtime.
* **Rationale**: 
  * **Performance**: avoid calling an LLM at runtime for every preview. 
  * **Consistency**: ensures the agent sees the same description for the same part every time.
  * **Maintainability**: The JSON can be manually reviewed and edited (or "Gold Standard" created).
* **Alternatives**:
  * *Runtime LLM Generation*: Too slow and costly.
  * *Procedural Generation*: Possible for simple parts (e.g. "Cylinder radius R"), but hard for complex COTS parts (NEMA motors).

### 1.2. Part Import Instructions
* **Decision**: The `Part` entity will include a `recipe` field (string) containing a Python code template.
* **Rationale**: `bd_warehouse` parts are instantiated via Python classes. A template like `from bd_warehouse.motor import Nema; part = Nema("Nema17")` is the most direct way for the Agent to use it.
* **Implementation**: The `PartProvider` will generate this recipe string dynamically based on the part parameters.

### 1.3. Indexing Strategy
* **Decision**: In-memory `PartIndex` populated at startup.
* **Rationale**: The number of COTS parts is initially small (< 1000). A simple list of `Part` objects filtered by Python's `in` operator (substring search) is sub-millisecond and requires no external infrastructure (ElasticSearch, etc.).

## 2. Dependencies

* **Library**: `bd_warehouse`
* **Action**: Add to `pyproject.toml`.

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
