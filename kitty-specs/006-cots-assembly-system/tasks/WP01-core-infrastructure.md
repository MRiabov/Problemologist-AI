---
work_package_id: "WP01"
title: "Core Infrastructure"
lane: "done"
dependencies: []
subtasks:
  - T001
  - T002
  - T003
  - T004
  - T005
  - T006
agent: "Gemini"
shell_pid: "277555"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP01: Core Infrastructure

**Goal**: Establish the base classes, data structures, and the central `PartIndex`.
**Role**: Backbone for the COTS system.

## Subtasks

### T001: Add Dependency
**Purpose**: Install `bd_warehouse` library.
**Steps**:
1. Add `bd_warehouse` to `pyproject.toml` dependencies.
2. Run `uv sync` (or equivalent) to update lockfile.

### T002: Create Package Structure
**Purpose**: Set up the `src/cots` directory.
**Steps**:
1. Create `src/cots/__init__.py`.
2. Create `src/cots/core.py` (will hold Index and Entities).
3. Create `src/cots/providers/__init__.py`.
4. Create `src/cots/providers/base.py`.

### T003: Define Data Classes
**Purpose**: Define the entities used to pass data around.
**Location**: `src/cots/core.py`
**Content**:
- `PartSummary`: id (str), name (str), provider (str).
- `PartPreview`: id (str), image_path (str), description (str), metadata (dict), recipe (str).
- `Part` (Internal): id (str), factory (callable), params (dict).

### T004: Define Abstract Provider
**Purpose**: Define the interface for all part sources.
**Location**: `src/cots/providers/base.py`
**Content**:
- Abstract Base Class `PartProvider`.
- Abstract method `search(self, query: str) -> List[PartSummary]`.
- Abstract method `get_part(self, part_id: str) -> Part`.
- Abstract method `get_preview(self, part_id: str) -> PartPreview`.

### T005: Implement PartIndex
**Purpose**: The central registry.
**Location**: `src/cots/core.py`
**Content**:
- Class `PartIndex`.
- Method `register_provider(self, provider: PartProvider)`.
- Method `search(self, query: str) -> List[PartSummary]`: Aggregates results from all providers.
- Method `preview(self, part_id: str) -> PartPreview`: Delegates to the correct provider (parse prefix `provider_name:` from ID).

### T006: Unit Tests
**Purpose**: Verify the plumbing works.
**Location**: `tests/test_cots_core.py`
**Steps**:
1. Implement a `MockProvider`.
2. Test registration.
3. Test search aggregation.
4. Test preview delegation.

## Definition of Done
- `bd_warehouse` is installed.
- Core classes exist and are importable.
- `PartIndex` can register a mock provider and route calls to it.
- Tests pass.

## Activity Log

- 2026-02-01T12:03:26Z – Gemini – shell_pid=264547 – lane=doing – Started implementation via workflow command
- 2026-02-01T12:05:51Z – Gemini – shell_pid=264547 – lane=for_review – Core infrastructure for COTS system implemented. Includes PartIndex, PartProvider base class, and unit tests.
- 2026-02-01T12:14:36Z – Gemini – shell_pid=277555 – lane=doing – Started review via workflow command
- 2026-02-01T12:18:58Z – Gemini – shell_pid=277555 – lane=done – Review passed: Core infrastructure implemented correctly, tests passing.
