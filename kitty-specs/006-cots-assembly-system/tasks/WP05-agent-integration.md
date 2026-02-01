---
work_package_id: "WP05"
title: "Agent Integration"
lane: "planned"
dependencies: ["WP04"]
subtasks:
  - T020
  - T021
  - T022
---

# WP05: Agent Integration

**Goal**: Expose the functionality to the Agent via Tools.
**Role**: The final interface layer.

## Subtasks

### T020: Register Search Tool
**Purpose**: Expose `search_parts`.
**Location**: `src/environment/tools.py`
**Steps**:
1. Initialize `PartIndex` (singleton).
2. Define tool function `search_parts(query: str)`.
3. Docstring: "Search for COTS parts by name. Returns list of matches."
4. Register with environment.

### T021: Register Preview Tool
**Purpose**: Expose `preview_part`.
**Location**: `src/environment/tools.py`
**Steps**:
1. Define tool function `preview_part(part_id: str)`.
2. Docstring: "Get visual preview and details for a part ID."
3. Register with environment.

### T022: Integration Test
**Purpose**: End-to-end verification.
**Location**: `tests/test_cots_integration.py`
**Steps**:
1. Test script that simulates Agent behavior:
   - Call `search_parts("Nema")`.
   - Pick first result ID.
   - Call `preview_part(id)`.
   - Check that image exists on disk and description is text.

## Definition of Done
- Tools are callable from `Environment`.
- Integration test passes.
- Feature is ready for use.
