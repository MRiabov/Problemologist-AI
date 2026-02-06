---
work_package_id: "WP02"
title: "Validation & Worker Utilities"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T005", "T006", "T007", "T008"]
---

# WP02: Validation & Worker Utilities

**Goal**: Implement the physics and design validation logic that runs on the worker container.

## Subtasks

### T005: Implement `simulate()`

**Purpose**: Provide a physics-backed stability check.

**Instructions**:

1. Implement `simulate(component: Compound) -> SimulationResult` in `src/worker/utils/validation.py`.
2. Logic:
   - Convert `Compound` to MJCF.
   - Run MuJoCo for a few frames.
   - Assert no NaNs/explosions.
   - Generate standard 24-view renders in `/renders/`.
   - Return stability status and render paths.

### T006: Implement `validate()`

**Purpose**: Verify geometric validity and randomization robustness.

**Instructions**:

1. Implement `validate(component: Compound) -> bool` in `src/worker/utils/validation.py`.
2. Logic:
   - Check for part intersections.
   - Verify boundary constraints (AABB).
   - Test validity across a few random seeds.

### T007: Implement `submit_for_review()`

**Purpose**: Standardized handover from Coder to Reviewer.

**Instructions**:

1. Implement `submit_for_review(component: Compound)` in `src/worker/utils/handover.py`.
2. Logic:
   - Persist temporary assets to the `/renders/` folder.
   - Trigger a LangGraph event or update shared state for the Reviewer node.

### T008: Expose Tools to Agent

**Purpose**: Register utilities as tools callable by LLM.

**Instructions**:

1. In `src/worker/utils/filesystem.py`, configure `FilesystemMiddleware` with `SandboxFilesystemBackend`.
2. Register `simulate`, `validate`, and `submit_for_review` as available tools in the `deepagents` environment.

## Verification

- [ ] `simulate()` correctly identifies unstable simulations.
- [ ] `validate()` catches overlapping parts or out-of-bounds geometry.
- [ ] `submit_for_review()` correctly triggers the next stage in the graph.
- [ ] Agent can successfully call these tools in a test sandbox.
