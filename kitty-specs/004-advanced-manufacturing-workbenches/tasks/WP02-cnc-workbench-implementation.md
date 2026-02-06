---
work_package_id: "WP02"
title: "CNC Workbench Implementation"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T005", "T006", "T007", "T008", "T009"]
---

# WP02: CNC Workbench Implementation

## Goal

Implement the CNC-specific validation logic (raycasting, corner radii) and cost estimation.

## Context

Implement `CNCWorkbench` which checks if a part is millable (3-axis) and estimates its cost.

## Subtasks

### T005: Implement CNCWorkbench Class Skeleton

**Objective**: Create the class structure.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Create `src/workbenches/cnc.py`.
2. Import `Workbench`, `WorkbenchResult` from base/models.
3. Define `CNCWorkbench(Workbench)`.
4. Implement `analyze` method stub.

### T006: Implement Geometry Validation (Visibility)

**Objective**: Check for undercuts using raycasting.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. In `analyze`:
2. Convert `build123d` part to `trimesh` mesh.
3. Implement `_check_visibility(mesh)`:
   - For 3-axis (Z-axis), check if all upward-facing surfaces are visible from above.
   - Alternatively/Simply: Check if any face normal points downwards (Z < 0) AND is occluded?
   - Strategy: "Heightmap" check or simple undercut check.
   - Simplest robust check: Project mesh to XY plane. Check if multiple Z values exist for same XY = undercut.
   - Log violations if undercuts found.

### T007: Implement Internal Corner Radius Check

**Objective**: Ensure no sharp internal corners.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Implement `_check_internal_radii(part)`:
   - Iterate over edges.
   - Identify concave edges (internal).
   - Check if radius < tool_radius (default 3mm or from config).
   - `build123d` topology analysis required.
   - If sharp internal corner found, add violation.

### T008: Implement CNC Cost Estimation

**Objective**: Calculate cost based on volume and time.
**Files**: `src/workbenches/cnc.py`
**Instructions**:

1. Implement `_calculate_cost(part, material_cost_per_kg)`:
   - Calculate bounding box volume (Material block size).
   - Material Cost = Block Volume *Density* Cost/kg.
   - Machining Time = Removed Volume / MRR (Material Removal Rate) + Setup Time.
   - Machining Cost = Time * Hourly Rate.
   - Total Cost = Material + Machining.
2. Return details in `WorkbenchResult`.

### T009: Write Comprehensive Tests

**Objective**: Verify logic.
**Files**: `tests/workbenches/test_cnc.py`
**Instructions**:

1. Create `tests/workbenches/test_cnc.py`.
2. Test Case 1: Simple Box (Valid).
3. Test Case 2: Box with hole (Valid).
4. Test Case 3: "T" shape from side (Undercut - Invalid).
5. Test Case 4: Box with sharp internal pocket (Invalid radius).
6. Verify cost calculations are non-zero and logical.

## Verification

- Run `pytest tests/workbenches/test_cnc.py`.

## Definition of Done

- `CNCWorkbench` implemented.
- Undercuts detected.
- Sharp internal corners detected.
- Cost calculated.
- Tests pass.
