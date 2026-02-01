---
work_package_id: "WP03"
title: "CNC Workbench Implementation"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T010", "T011", "T012", "T013"]
---

### Objective
Implement the `CNCWorkbench` class that orchestrates DFM checks and cost estimation for 3-axis milling.

### Context
This class inherits from `src.workbenches.base.Workbench` (check existing code for signature). It uses the utils from WP02 to validate parts.

### Subtask T010: Class Skeleton
**Purpose**: Create the workbench file and class structure.
**Steps**:
1.  Create `src/workbenches/cnc.py`.
2.  Define `class CNCWorkbench(Workbench):`.
3.  Implement `__init__` to load specific config from `manufacturing_config.yaml` (using `load_config` from utils).

### Subtask T011: CNC Validation Logic
**Purpose**: Implement specific checks for milling.
**Steps**:
1.  Implement `validate_geometry(self, part: Part) -> ValidationResult`.
2.  Convert part to mesh using `part_to_trimesh`.
3.  **Check 1: Undercuts**. Call `check_undercuts` with `pull_vector=(0,0,1)` (Z-axis assumption).
4.  **Check 2: Tool Access**. (Optional/Advanced) Check for sharp internal corners if feasible, or skip for MVP. Focus on Undercuts first.
5.  Return a structured result (list of violations).

### Subtask T012: CNC Cost Model
**Purpose**: Calculate the price based on volume and machine time.
**Steps**:
1.  Implement `calculate_cost(self, part: Part, quantity: int) -> CostAnalysis`.
2.  **Material**: `part.volume` * density * material_price.
3.  **Run Time**: `part.volume` / removal_rate * machine_hourly_rate (Simplified).
4.  **Setup**: Fixed cost (e.g., 1 hour machine time) / quantity.
5.  Return `CostAnalysis` dict (unit_cost, total_cost, breakdown).

### Subtask T013: CNC Workbench Tests
**Purpose**: Verify the workbench logic.
**Steps**:
1.  Create `tests/test_workbench_cnc.py`.
2.  Test `validate_geometry`: Pass a Cube (valid) and a Mushroom/T-shape (invalid undercut). Assert correct violations.
3.  Test `calculate_cost`: Pass a known volume, check if math matches the config values.

### Definition of Done
*   `CNCWorkbench` implemented and working.
*   Correctly identifies undercuts.
*   Returns plausible cost estimates.
