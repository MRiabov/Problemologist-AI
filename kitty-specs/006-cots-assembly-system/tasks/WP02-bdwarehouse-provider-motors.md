---
work_package_id: WP02
title: BDWarehouse Provider (Motors)
lane: "done"
dependencies: [WP01]
base_branch: main
base_commit: 32004050454b972640e502e825b4ae7497e2407e
created_at: '2026-02-01T12:30:45.932013+00:00'
subtasks:
- T007
- T008
- T009
- T010
shell_pid: "332564"
agent: "gemini-cli"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP02: BDWarehouse Provider (Motors)

**Goal**: Implement the `BDWarehouseProvider` and support indexing NEMA Motors.
**Role**: The first real implementation of a provider.

## Subtasks

### T007: Provider Skeleton
**Purpose**: Create the specific provider class.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Create `BDWarehouseProvider` inheriting from `PartProvider`.
2. Implement `__init__` to initialize an empty index/cache.

### T008: Index NEMA Motors
**Purpose**: Crawl `bd_warehouse` for available motors.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Import `Nema` from `bd_warehouse.motor`.
2. Use `Nema.sizes()` (or equivalent API) to get list of sizes (Nema17, Nema23, etc.).
3. Populate the internal index with `Part` objects for each size.
4. IDs should look like: `bd_warehouse:motor:Nema17`.

### T009: Recipe Generation
**Purpose**: Generate Python code to instantiate the part.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Implement `get_part` to return the `Part` object.
2. Implement `get_preview` (skeleton for now, focusing on recipe).
3. Recipe format:
   ```python
   from bd_warehouse.motor import Nema
   part = Nema("Nema17")
   ```

### T010: Unit Tests
**Purpose**: Verify motors are indexed.
**Location**: `tests/test_cots_bdwarehouse.py`
**Steps**:
1. Instantiate `BDWarehouseProvider`.
2. Call `search("Nema")`.
3. Assert "Nema17" is in results.
4. Check the recipe string for correctness.

## Definition of Done
- `BDWarehouseProvider` exists.
- NEMA motors are searchable.
- Correct recipes are generated.
- Tests pass.

## Activity Log

- 2026-02-01T12:39:59Z – unknown – shell_pid=288427 – lane=for_review – Ready for review: Implemented BDWarehouseProvider with NEMA motor support and tests.
- 2026-02-01T13:06:44Z – Antigravity – shell_pid=286014 – lane=doing – Started review via workflow command
- 2026-02-01T13:10:59Z – Antigravity – shell_pid=286014 – lane=done – Review passed: BDWarehouseProvider implemented with NEMA motor support. Tests pass and recipe generation is correct. Correctly adapted to actual bd_warehouse library structure.
- 2026-02-01T13:11:37Z – Antigravity – shell_pid=315558 – lane=doing – Started implementation via workflow command
- 2026-02-01T13:32:50Z – Antigravity – shell_pid=315558 – lane=for_review – Ready for review: Implemented BDWarehouseProvider with NEMA motor support and tests.
- 2026-02-01T13:35:40Z – gemini-cli – shell_pid=332564 – lane=doing – Started review via workflow command
- 2026-02-01T13:42:24Z – gemini-cli – shell_pid=332564 – lane=done – Review passed: BDWarehouseProvider implemented with NEMA motor support and verified with unit tests.
