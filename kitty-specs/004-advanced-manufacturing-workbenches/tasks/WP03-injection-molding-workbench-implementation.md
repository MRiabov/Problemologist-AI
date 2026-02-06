---
work_package_id: "WP03"
title: "Injection Molding Workbench Implementation"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T010", "T011", "T012", "T013", "T014"]
---

# WP03: Injection Molding Workbench Implementation

## Goal

Implement the Injection Molding validation logic (draft angles, wall thickness) and cost estimation.

## Context

Implement `InjectionMoldingWorkbench` to check for IM feasibility (drafts, wall thickness) and estimate mold/part costs.

## Subtasks

### T010: Implement Workbench Class Skeleton

**Objective**: Create the IM workbench class.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Create `src/workbenches/injection_molding.py`.
2. Import `Workbench`, `WorkbenchResult` from base/models.
3. Define `InjectionMoldingWorkbench(Workbench)`.
4. Implement `analyze` method stub.

### T011: Implement Draft Angle Validation

**Objective**: Ensure vertical faces have sufficient draft.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. In `analyze`:
2. Convert to mesh (trimesh).
3. Implement `_check_draft_angles(mesh, min_angle=2.0)`:
   - Identify faces that are roughly vertical (relative to Pull Direction, usually Z).
   - Calculate angle between face normal and Pull Vector.
   - If angle < min_angle (and not 0/90 perfectly?): Violation.
   - Note: 90 deg = vertical. Need > 2 deg draft, so angle should be > 92 or < 88?
   - Logic: `abs(dot(normal, pull_vector))` should not be close to 0 (which would be perpendicular to pull). Wait.
   - Normal of a vertical face is perpendicular to Z. Dot product is 0.
   - We want normal to slightly point up (or down).
   - Logic: `acos(abs(normal.z))` converted to degrees.
   - If `90 - angle < 2.0`: Violation.

### T012: Implement Wall Thickness Verification

**Objective**: Check for thin/thick walls.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Implement `_check_wall_thickness(mesh)`:
   - Robust method: Ray casting from random surface points inward along normal.
   - If ray hits "backside" face: measure distance.
   - If distance < min_thickness OR distance > max_thickness (variance): Warning/Violation.
   - Implement simple sampling (e.g., 100 points).

### T013: Implement IM Cost Estimation

**Objective**: Calculate mold and part cost.
**Files**: `src/workbenches/injection_molding.py`
**Instructions**:

1. Implement `_calculate_cost(part, quantity)`:
   - Mold Base Cost (Fixed): Expensive (e.g., $5000).
   - Material Cost: Volume *Density* Cost/kg.
   - Processing Cost: Cycle time * Hourly rate.
   - Total = Mold Cost + (Material + Processing) * Quantity.
   - Unit Cost = Total / Quantity.
2. Return details in `WorkbenchResult`.

### T014: Write Comprehensive Tests

**Objective**: Verify IM logic.
**Files**: `tests/workbenches/test_im.py`
**Instructions**:

1. Create `tests/workbenches/test_im.py`.
2. Test Case 1: Cube (Valid? No, vertical walls need draft).
3. Test Case 2: Pyramid/Trapezoid (Valid draft).
4. Test Case 3: Thin wall plate (Valid).
5. Test Case 4: Thick block (Potential sink mark warning -> maybe strict violation for now).
6. Verify cost amortization (Unit cost drops with quantity).

## Verification

- Run `pytest tests/workbenches/test_im.py`.

## Definition of Done

- `InjectionMoldingWorkbench` implemented.
- Draft angles checked.
- Cost model implemented.
- Tests pass.
