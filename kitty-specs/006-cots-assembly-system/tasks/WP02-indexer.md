---
work_package_id: WP02
title: Indexer Implementation
lane: "for_review"
dependencies: [WP01]
base_branch: 006-cots-assembly-system-WP01
base_commit: 22f6e149ae2d21566c7afdae80a281f72989dcbf
created_at: '2026-02-06T15:07:15.564745+00:00'
subtasks: [T005, T006, T007, T008, T009]
shell_pid: "684091"
agent: "Gemini-CLI"
---

# WP02: Indexer Implementation

**Goal**: Build the crawler that populates our parts database from `bd_warehouse`.

## Subtasks

### T005: Implement `bd_warehouse` crawler

**Purpose**: Discover all part classes in `bd_warehouse`.
**File**: `src/cots/indexer.py` (or helper)
**Steps**:

- Import `bd_warehouse`.
- Inspect the module to find subclasses of `Part` (or whatever the base class is in `bd_warehouse`, likely need exploration).
- Recursively find all fasteners, motors, etc.

### T006: Implement Metadata Extractor

**Purpose**: Instantiate parts and extract physical properties.
**File**: `src/cots/indexer.py`
**Details**:

- For a given part class (e.g. `HexNut`) and parameters (e.g. `size='M6'`):
  - Instantiate the object: `part = HexNut('M6')`.
  - Extract:
    - `name`: e.g. "Hex Nut M6"
    - `bounding_box`: logic to find size.
    - `weight`: if available, or estimate.
    - `unit_cost`: Use a default cost model or placeholder logic (e.g. $0.10 for screws).

### T007: Implement "Recipe Generator"

**Purpose**: Create the exact Python string to recreate this object.
**File**: `src/cots/indexer.py`
**Details**:

- Needs to produce a string like `"from bd_warehouse import HexNut; part = HexNut('M6')"`
- This allows the agent to just `exec()` or paste this code.

### T008: Indexer Main Loop

**Purpose**: Orchestrate the indexing.
**File**: `src/cots/indexer.py`
**Steps**:

- `main()` function:
  - Initialize DB using `init_db` (from WP01).
  - Loop through defined categories/parts.
  - Extract metadata and recipe.
  - Save `COTSItem` to DB via ORM.
  - Handle errors gracefully (log skip if instantiation fails).

### T009: Unit Test for Indexer

**Purpose**: Ensure it works without running for hours.
**File**: `tests/cots/test_indexer.py`
**Steps**:

- Mock `bd_warehouse` or use a very simple part.
- Run indexer logic.
- Verify DB contains the item.

## Validation

- [ ] Indexer runs and creates `parts.db`.
- [ ] `parts.db` contains expected parts.
- [ ] Recipes are valid Python code.

## Implementation Command

`spec-kitty implement WP02 --base WP01`

## Activity Log

- 2026-02-06T15:07:15Z – Gemini-CLI – shell_pid=684091 – lane=doing – Assigned agent via workflow command
- 2026-02-06T15:11:13Z – Gemini-CLI – shell_pid=684091 – lane=for_review – Implemented COTS Indexer. Crawls bd_warehouse for fasteners and bearings, extracts metadata (bbox, volume, weight, cost), and generates import recipes. Verified with unit tests.
