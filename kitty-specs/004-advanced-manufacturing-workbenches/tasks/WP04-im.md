---
work_package_id: WP04
title: Injection Molding Workbench Implementation
lane: "for_review"
dependencies: "[]"
base_branch: 004-advanced-manufacturing-workbenches-WP02
base_commit: 58830b6ba99207131bb366fcf00bae5a3246b61a
created_at: '2026-02-01T11:31:06.257252+00:00'
subtasks: [T014, T015, T016, T017]
shell_pid: "231429"
agent: "gemini-cli-agent"
assignee: "gemini-cli-agent"
---

### Objective

Implement the `InjectionMoldingWorkbench` class for high-volume plastic part validation.

### Context

Inherits from `src.workbenches.base.Workbench`. Focuses on Draft angles and Moldability.

### Subtask T014: Class Skeleton

**Purpose**: Create the workbench file and class.
**Steps**:

1. Create `src/workbenches/injection_molding.py`.
2. Define `class InjectionMoldingWorkbench(Workbench):`.
3. Load config in `__init__`.

### Subtask T015: IM Validation Logic

**Purpose**: Check for moldability.
**Steps**:

1. Implement `validate_geometry(self, part: Part) -> ValidationResult`.
2. Convert to mesh.
3. **Check 1: Draft**. Call `check_draft_angle` with config's `min_draft_angle`.
4. **Check 2: Undercuts**. Call `check_undercuts`.
5. **Check 3: Thickness**. Call `check_wall_thickness`.
6. Collect all violations into the report.

### Subtask T016: IM Cost Model

**Purpose**: Calculate tooling vs unit cost trade-off.
**Steps**:

1. Implement `calculate_cost(self, part: Part, quantity: int)`.
2. **Tooling (NRE)**: High fixed cost. Base cost + (Surface Area * complexity_factor).
3. **Material**: `part.volume` *density* material_price.
4. **Cycle Cost**: (Volume / injection_rate) * machine_rate.
5. Note: At low quantity (e.g., 10), unit cost should be astronomical due to tooling. At high quantity (10k), it should be low.

### Subtask T017: IM Workbench Tests

**Purpose**: Verify IM logic.
**Steps**:

1. Create `tests/test_workbench_im.py`.
2. Test `validate_geometry`:
    * Box with vertical walls -> Fail (Draft).
    * Pyramid -> Pass.
    * Thick block -> Fail (Thickness).
3. Test `calculate_cost`: Verify high startup cost behavior.

### Definition of Done

* `InjectionMoldingWorkbench` implemented.
* Catches draft, undercut, and thickness issues.
* Cost model reflects economies of scale.

## Activity Log

* 2026-02-01T11:33:27Z – unknown – shell_pid=231429 – lane=for_review – Injection Molding Workbench implemented with Draft, Undercut, and Thickness validation.
* 2026-02-01T14:45:15Z – unknown – shell_pid=231429 – lane=for_review – Cleaned up garbage files from the root directory.
