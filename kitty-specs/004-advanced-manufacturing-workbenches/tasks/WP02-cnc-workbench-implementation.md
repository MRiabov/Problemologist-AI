---
work_package_id: "WP02"
title: "CNC Workbench Implementation"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T005", "T006", "T007", "T008", "T009"]
---

# WP02: CNC Workbench Implementation

## Goal

Implement the CNC-specific functional validation logic (undercuts, corner radii) and cost estimation.

## Context

Implement the `analyze_cnc` function which checks if a part is millable (3-axis) and estimates its cost using functional helpers and `structlog`.

## Subtasks

### T005: Implement CNC Analysis Skeleton

**Objective**: Create the functional module structure.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Create `src/workbenches/cnc.py`.
2. Define `analyze_cnc(part: Part | Compound, config: CNCConfig) -> WorkbenchResult`.
3. Use `structlog` to log the start of the analysis.

### T006: Implement Undercut Validation

**Objective**: Check for undercuts using raycasting and functional helpers.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Implement `_check_undercuts(mesh: trimesh.Trimesh) -> List[str]`.
2. Use `structlog` to trace raycasting progress and log any found undercuts.
3. Integrate into `analyze_cnc`.

### T007: Implement Internal Corner Radius Check

**Objective**: Functional check for sharp internal corners.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Implement `_check_internal_radii(part: Part | Compound, tool_radius: float) -> List[str]`.
2. Use `structlog` to log edge analysis.
3. Integrate into `analyze_cnc`.

### T008: Implement CNC Cost Estimation

**Objective**: Functional cost calculation.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Implement `_calculate_cnc_cost(part, material_config, cnc_config) -> float`.
2. Log cost breakdown (material vs machining) with `structlog`.
3. Return details in `WorkbenchResult`.

### T009: Write CNC Functional Tests

**Objective**: Verify logic with unit tests.
**Files**: `tests/workbenches/test_cnc.py`
**Instructions**:

1. Create `tests/workbenches/test_cnc.py`.
2. Test `analyze_cnc` with various geometries.
3. Verify `structlog` entries for failures.

## Verification

- Run `pytest tests/workbenches/test_cnc.py`.

## Definition of Done

- `analyze_cnc` function implemented.
- Undercuts and corner radii checks functional.
- `structlog` integrated throughout.
- Cost estimated and Pydantic-validated.
- Tests pass.
