# Research: COTS Assembly System

## Catalog Lookup Strategy

The Engineering Agent requires access to Off-The-Shelf (COTS) parts (motors, fasteners, gears) without cluttering its primary context with thousands of part definitions.

### 1. The Searching Subagent

**Decision**: Use a dedicated **Catalog Subagent**.
**Rationale**:

- The main Agent (Engineer/Planner) only prompts the searching subagent with requirements (e.g., "M6 bolt", "NEMA 17 motor").
- The subagent executes read-only SQL queries or structured Grep over a catalog database.
- Reduces primary agent token usage and improves reliability of part selection.

### 2. Catalog Persistence

**Decision**: S3-hosted JSON/SQLite catalog.
**Rationale**:

- Standardized `bd_warehouse` parts are indexed into a high-performance JSON/SQLite file.
- The subagent uses this index to return a set of matching part IDs and metadata.

### 3. Integration via ReAct

**Decision**: Python-native instantiation.
**Rationale**:

- Once a part is selected, the subagent returns a Python "recipe" (e.g., `HexNut('M6')`).
- The Engineer Agent imports these parts directly into its `script.py`, maintaining the "Code-as-Policy" paradigm.

### 4. Manufacturability of Assemblies

**Decision**: Non-interfering constraint checks.
**Rationale**:

- COTS parts are treated as rigid, read-only compounds.
- The `validate_and_price` tool ensures COTS parts do not interfere with custom CAD geometry and that constraints (e.g., bolt clearance) are respected.
