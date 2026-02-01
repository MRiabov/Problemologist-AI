---
work_package_id: WP05
title: Agent Integration
lane: "for_review"
dependencies: [WP04]
base_branch: main
base_commit: dba744bf6522a47a4f6f52a17f93666893fae774
created_at: '2026-02-01T17:34:48.018398+00:00'
subtasks:
- T020
- T021
- T022
shell_pid: "490041"
review_status: "has_feedback"
reviewed_by: "MRiabov"
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

## Activity Log

- 2026-02-01T18:22:39Z – unknown – shell_pid=490041 – lane=for_review – Ready for review: Registered COTS search and preview tools in Environment and Agent layers, and verified with integration tests.
- 2026-02-01T18:32:38Z – unknown – shell_pid=490041 – lane=planned – Moved to planned
- 2026-02-01T18:40:11Z – unknown – shell_pid=490041 – lane=for_review – Ready for review: Registered COTS search and preview tools in Environment and Agent layers, and verified with integration tests.
