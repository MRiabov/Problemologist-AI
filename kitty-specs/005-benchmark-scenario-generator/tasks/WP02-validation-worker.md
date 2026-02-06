---
work_package_id: "WP02"
title: "Validation & Worker Utilities"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T005", "T006", "T007", "T008"]
---

# WP02: Validation & Worker Utilities

**Goal**: Implement the physics simulation and validation logic that runs on the Worker node.

## Subtasks

### T005: Implement Stability Simulation

**Purpose**: Run a headless MuJoCo simulation to verify the generated model does not explode or contain invalid geometry.

**Instructions**:

1. Create `src/worker/utils/validation.py`.
2. Implement `simulate_stability(mjcf_content: str, duration: float = 2.0) -> ValidationResult`:
   - Load MJCF string into `mujoco.MjModel`.
   - Create `mujoco.MjData`.
   - Step simulation for `duration`.
   - **Checks**:
     - Check for `nan` (divergence) in `qpos` or `qvel`.
     - Check for excessive energy (explosion).
     - Check for unwanted collisions (if "forbid" zones are defined - interpret from MJCF or args).
   - Return structured `ValidationResult` (success: bool, error_message: str, logs: List[str]).

### T006: Headless Rendering Utility

**Purpose**: Generate 24 multi-view images for the Reviewer.

**Instructions**:

1. Add `render_views(mjcf_content: str, output_dir: str)` to `src/worker/utils/rendering.py`.
2. Implementation:
   - Initialize MuJoCo with an offscreen GL context (e.g., `mujoco.GLContext`).
   - Setup camera:
     - 45 degree elevation.
     - 8 azimuth angles (0, 45, 90, ...).
     - 3 zoom levels or 3 different elevations if preferred (Spec says "8 pictures on 3 levels" -> 24 images).
   - Render frames and save as PNGs in `output_dir`.
   - Bundle into a ZIP file if needed, or return list of paths.

### T007: Unit Tests for Validation

**Purpose**: Ensure validation catches bad models.

**Instructions**:

1. Create `tests/worker/test_validation.py`.
2. Create:
   - A valid MJCF (simple box sitting on plane).
   - An invalid MJCF (two boxes overlapping heavily -> explosion).
3. Test `simulate_stability` against both. Ensure it returns True for valid and False for invalid.
4. Test `render_views` generates files (mock GL context if testing environment lacks GPU/display, or skip if strictly dependent on EGL/GLX).

### T008: Expose Validation Tools

**Purpose**: Make these accessible as Tools for the Agent.

**Instructions**:

1. In `src/worker/tools.py` (or equivalent registry), register:
   - `validate_mjcf` (wraps `simulate_stability`)
   - `render_mjcf` (wraps `render_views`)

## Verification

- [ ] `simulate_stability` correctly identifies unstable simulations.
- [ ] `render_views` produces 24 images in the specified directory.
- [ ] Tests pass in the local environment (or docker container).
