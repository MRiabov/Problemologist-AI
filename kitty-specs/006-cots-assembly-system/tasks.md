# Tasks: COTS Assembly System

**Feature**: 006-cots-assembly-system
**Status**: Planned

## Work Packages

### WP01: Data Core & Foundation

**Summary**: Define the Pydantic models, SQLAlchemy schema, and basic infrastructure for the COTS system.
**Priority**: High
**Dependencies**: None
**Estimated Size**: ~250 lines

- [ ] T001: Create project structure `src/cots/` and `src/cots/database/`
- [ ] T002: Define `COTSItem` and `SearchQuery` Pydantic models in `src/cots/models.py`
- [ ] T003: Define SQLAlchemy ORM models in `src/cots/database/models.py` matching `COTSItem`
- [ ] T004: Create database initialization script `src/cots/database/init.py`

### WP02: Indexer Implementation

**Summary**: Implement the build-time indexer that scrapes `bd_warehouse` and populates the SQLite database.
**Priority**: High
**Dependencies**: WP01 (Data Models)
**Estimated Size**: ~400 lines

- [ ] T005: Implement `bd_warehouse` crawler to discover parts
- [ ] T006: Implement metadata extractor (extract properties from `build123d` objects)
- [ ] T007: Implement "Recipe Generator" (generate import/instantiation code)
- [ ] T008: Implement `src/cots/indexer.py` main logic to process parts and save to DB
- [ ] T009: Create unit test for indexer with a mock/small part subset

### WP03: Search Runtime

**Summary**: Implement the runtime search functionality and expose it as a tool/subagent.
**Priority**: Medium
**Dependencies**: WP02 (Populated Database)
**Estimated Size**: ~200 lines

- [ ] T010: Implement `search_parts` function in `src/cots/runtime.py` with SQL filtering
- [ ] T011: Create the "Search COTS" tool definition (LangChain/DeepAgents compatible)
- [ ] T012: Create integration test: Index a dummy part and search for it
