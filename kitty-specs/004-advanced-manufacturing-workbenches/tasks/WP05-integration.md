---
work_package_id: "WP05"
title: "Tool Integration & Caching"
lane: "planned"
dependencies: ["WP03", "WP04"]
subtasks: ["T018", "T019", "T020", "T021"]
---

### Objective
Expose the new analysis capabilities to the AI agent via a unified tool, ensuring performance via caching.

### Context
The agent shouldn't instantiate workbenches directly. It calls `check_manufacturability`. This tool needs to parse the request, route to the right workbench, and cache the result because raycasting is slow.

### Subtask T018: Update Tools Module
**Purpose**: Import the new workbenches.
**Steps**:
1.  Open `src/environment/tools.py`.
2.  Import `CNCWorkbench` and `InjectionMoldingWorkbench`.
3.  Instantiate them as module-level singletons or within a factory (to keep config loaded).

### Subtask T019: Implement Cached Tool Function
**Purpose**: Create the main entry point.
**Steps**:
1.  Define `check_manufacturability(design_file: str, process: str, quantity: int) -> dict`.
2.  **Caching Logic**:
    *   Since `design_file` path might change content, we can't cache on path alone.
    *   Read file content or compute hash of the file.
    *   Use `@functools.lru_cache` on a helper function `_analyze_internal(file_hash, process, quantity)`.
3.  **Routing**:
    *   If `process == "cnc"`, use CNC workbench.
    *   If `process == "injection_molding"`, use IM workbench.
    *   Else return error.

### Subtask T020: Enforce Output Contract
**Purpose**: Ensure output JSON matches the schema.
**Steps**:
1.  Map the internal `ValidationResult` and `CostAnalysis` objects to the dictionary structure defined in `contracts/tool_schema.json`.
2.  Ensure keys like `status`, `violations`, `cost_analysis` are always present.

### Subtask T021: Integration Test
**Purpose**: Verify the agent can actually call this.
**Steps**:
1.  Create `tests/test_tool_integration.py`.
2.  Mock the file system (or use tmp file).
3.  Call `check_manufacturability`.
4.  Assert it runs end-to-end and returns the correct JSON structure.
5.  Verify caching: Call twice, check logs/mock to ensure analysis only ran once.

### Definition of Done
*   `check_manufacturability` is available in `tools.py`.
*   Caching works (second call is instant).
*   Output format matches schema.
