---
work_package_id: WP03
title: Injection Molding Workbench Implementation
lane: "for_review"
dependencies: [WP01]
base_branch: 004-advanced-manufacturing-workbenches-WP02
base_commit: 11908c76e29b8964dd73e20485f87c80a221329f
created_at: '2026-02-06T21:08:41.118506+00:00'
subtasks: [T010, T011, T012, T013, T014]
shell_pid: "119564"
agent: "Gemini"
---

# WP03: Injection Molding Workbench Implementation

## Goal

Implement the Injection Molding functional validation logic (draft angles, wall thickness) and cost estimation.

## Context

Implement the `analyze_im` function which checks if a part is moldable and estimates its cost using functional helpers and `structlog`.

## Subtasks

### T010: Implement IM Analysis Skeleton

**Objective**: Create the functional module structure.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Create `src/workbenches/injection_molding.py`.
2. Define `analyze_im(part: Part | Compound, config: IMConfig) -> WorkbenchResult`.
3. Use `structlog` to log the start of the analysis.

### T011: Implement Draft Angle Validation

**Objective**: Functional check for draft angles.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Implement `_check_draft_angles(mesh: trimesh.Trimesh, pull_vector: Tuple[float, float, float]) -> List[str]`.
2. Use `structlog` to trace normal analysis.
3. Integrate into `analyze_im`.

### T012: Implement Wall Thickness Verification

**Objective**: Functional check for wall thickness consistency.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Implement `_check_wall_thickness(mesh: trimesh.Trimesh) -> List[str]`.
2. Use `structlog` to log raycast sampling progress.
3. Integrate into `analyze_im`.

### T013: Implement IM Cost Estimation

**Objective**: Functional cost calculation.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Implement `_calculate_im_cost(part, material_config, im_config) -> float`.
2. Log mold base vs cycle time costs with `structlog`.
3. Return details in `WorkbenchResult`.

### T014: Write IM Functional Tests

**Objective**: Verify logic with unit tests.
**Files**: `tests/workbenches/test_im.py`
**Instructions**:

1. Create `tests/workbenches/test_im.py`.
2. Test `analyze_im` with various geometries.
3. Verify `structlog` entries for DFM violations.

## Verification

- Run `pytest tests/workbenches/test_im.py`.

## Definition of Done

- `analyze_im` function implemented.
- Draft angle and wall thickness checks functional.
- `structlog` integrated throughout.
- Cost estimated and Pydantic-validated.
- Tests pass.

## Activity Log

- 2026-02-06T21:08:41Z – Gemini – shell_pid=119564 – lane=doing – Assigned agent via workflow command
- 2026-02-06T21:13:19Z – Gemini – shell_pid=119564 – lane=for_review – Ready for review: Implemented Injection Molding workbench with functional analysis (analyze_im), draft angle and wall thickness validation, and cost estimation. Added unit tests verifying logic and structlog integration.
