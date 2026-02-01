---
work_package_id: WP01
title: Infrastructure & Validator
lane: "done"
dependencies: []
subtasks:
- T001
- T002
- T003
agent: "Gemini"
shell_pid: "353112"
reviewed_by: "MRiabov"
review_status: "approved"
---

## Objective

Establish the foundation for the Benchmark Scenario Generator. This includes creating the directory structure, defining the data models, and implementing the critical "Validator" component that checks if generated physics scenes are stable.

## Context

We are building a standalone utility in `src/generators/benchmark/`. The validator is the "safety mechanism" that prevents the agent from generating broken or exploding simulations. It must run headless and fast.

## Subtasks

### T001: Scaffold Package Structure

**Purpose**: Create the module layout.
**Steps**:
1. Create directory `src/generators/benchmark/`.
2. Add `__init__.py` to make it a package.
3. Ensure `pyproject.toml` or environment has `mujoco` and `build123d` installed (already should be, but verify).

### T002: Define Data Models

**Purpose**: TypedDicts for type safety.
**Steps**:
1. Create `src/generators/benchmark/types.py` (or similar).
2. Define `ScenarioManifest` (JSON metadata structure).
3. Define `ValidationReport` (Stability check results).
4. Reference `data-model.md` for fields.

### T003: Implement Headless Validator

**Purpose**: The core validation logic.
**Steps**:
1. Create `src/generators/benchmark/validator.py`.
2. Implement `validate_mjcf(xml_string: str) -> ValidationReport`:
   - Load the XML string into `mujoco.MjModel.from_xml_string`.
   - Create `mujoco.MjData`.
   - Run a loop for 1.0 simulated second (usually 1000 steps at 1ms timestep).
   - Check for `NaN` coordinates or excessive velocities (>100m/s).
   - Return `passed=True` if stable.
3. Add basic error handling for XML syntax errors (return `passed=False`).

## Acceptance Criteria

- [ ] `src/generators/benchmark` exists and is importable.
- [ ] `validator.validate_mjcf` returns `False` for a broken XML.
- [ ] `validator.validate_mjcf` returns `True` for a simple valid XML (e.g., a static box).

## Activity Log

- 2026-02-01T12:03:17Z – gemini-cli – shell_pid=264295 – lane=doing – Started implementation via workflow command
- 2026-02-01T13:18:46Z – gemini-cli – shell_pid=264295 – lane=for_review – Implemented infrastructure, data models, and headless validator
- 2026-02-01T13:33:04Z – Antigravity – shell_pid=313636 – lane=doing – Started review via workflow command
- 2026-02-01T13:40:33Z – Antigravity – shell_pid=313636 – lane=done – Review passed: Infrastructure, Data Models, and Validator implemented and verified. Fixed energy calculation.
- 2026-02-01T13:53:15Z – Antigravity – shell_pid=313636 – lane=for_review – Implemented core infrastructure and MuJoCo validator for benchmark generator.
- 2026-02-01T13:55:13Z – Gemini – shell_pid=353112 – lane=doing – Started review via workflow command
- 2026-02-01T13:57:06Z – Gemini – shell_pid=353112 – lane=done – Review passed: Infrastructure, Data Models, and MuJoCo validator implemented and verified.
