# Work Packages: COTS Assembly System
*Path: kitty-specs/006-cots-assembly-system/tasks.md*

**Feature**: 006-cots-assembly-system
**Status**: Planned

## Summary
Implementation breakdown for the COTS Assembly System. The work is divided into 5 focused Work Packages to build the core infrastructure, the `bd_warehouse` provider, visual preview system, and agent integration.

---

## 🏗️ Phase 1: Infrastructure & Core

### WP01: Core Infrastructure
**Goal**: Establish the base classes, data structures, and the central `PartIndex`.
**Priority**: P0 (Blocker)
**Test**: Unit tests for `PartIndex` registration and search logic.

- [x] T001: Add `bd_warehouse` dependency to `pyproject.toml`
- [x] T002: Create `src/cots` package structure
- [x] T003: Define `Part`, `PartSummary`, `PartPreview` data classes
- [x] T004: Define abstract `PartProvider` base class
- [x] T005: Implement `PartIndex` (registration, search, preview orchestration)
- [x] T006: Write unit tests for `PartIndex` and data classes

**Implementation Notes**:
- Use Python's `dataclasses` for entities.
- `PartIndex` should be a singleton or easy to instantiate in `Environment`.
- `PartProvider` must handle the specific logic of fetching parts from a source.

**Dependencies**: None
**Est. Prompt Size**: ~350 lines

---

## 📦 Phase 2: Providers

### WP02: BDWarehouse Provider (Motors)
**Goal**: Implement the `BDWarehouseProvider` and support indexing NEMA Motors.
**Priority**: P0
**Test**: Unit tests verifying NEMA motors are found and correctly instantiated.

- [x] T007: Implement `BDWarehouseProvider` class skeleton
- [x] T008: Implement logic to crawl/index NEMA Motors from `bd_warehouse`
- [x] T009: Implement `get_part` logic to generate instantiation recipes for Motors
- [x] T010: Write unit tests for Motor indexing

**Implementation Notes**:
- Inspect `bd_warehouse` source or documentation to find how to list available parts (e.g., `Nema.sizes()`).
- Recipe generation should produce valid python code: `from bd_warehouse.motor import Nema; part = Nema("Nema17")`.

**Dependencies**: WP01
**Est. Prompt Size**: ~300 lines

### WP03: BDWarehouse Provider (Extended)
**Goal**: Extend the provider to support Bearings, Fasteners, and Beams.
**Priority**: P1
**Test**: Unit tests for each new category.

- [x] T011: Implement logic to index Bearings
- [x] T012: Implement logic to index Fasteners (Screws, Nuts)
- [x] T013: Implement logic to index Beams (T-Slot profiles)
- [x] T014: Ensure recipes are correct for each type

**Implementation Notes**:
- Fasteners have many variants (ISO, DIN). Ensure the index ID captures this (e.g., `bd_warehouse:fastener:iso4762:M6-10`).
- Beams are critical for structural frames.

**Dependencies**: WP02
**Est. Prompt Size**: ~300 lines

---

## 👁️ Phase 3: Preview & Integration

### WP04: Visual Preview System
**Goal**: Implement the rendering pipeline and description system.
**Priority**: P1
**Test**: Verify `preview_part` returns a valid image path and description.

- [x] T015: Create `src/assets/cots_descriptions.json` with initial data
- [x] T016: Implement helper to load/match descriptions from JSON
- [x] T017: Implement rendering logic (generate `build123d` object -> export STL/image)
- [x] T018: Integrate rendering into `PartIndex.preview`
- [x] T019: Handle caching of rendered images (optional but good)

**Implementation Notes**:
- Reuse existing rendering logic from Spec 001 if available (check `src/compiler` or similar).
- Descriptions should be looked up by Part ID (or partial match).

**Dependencies**: WP01, WP02
**Est. Prompt Size**: ~400 lines

### WP05: Agent Integration
**Goal**: Expose the functionality to the Agent via Tools.
**Priority**: P0
**Test**: End-to-end test of the tools in the environment.

- [x] T020: Register `search_parts` tool in `src/environment/tools.py`
- [x] T021: Register `preview_part` tool in `src/environment/tools.py`
- [ ] T022: Write integration test verifying Agent can call these tools

**Implementation Notes**:
- Ensure tool docstrings are descriptive so the LLM knows how to use them.
- Map tool errors (invalid ID) to clean strings, not tracebacks.

**Dependencies**: WP04
**Est. Prompt Size**: ~250 lines
