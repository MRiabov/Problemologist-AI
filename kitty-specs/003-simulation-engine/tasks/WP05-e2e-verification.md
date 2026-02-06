---
work_package_id: WP05
title: End-to-End Verification
lane: planned
dependencies: []
subtasks: [T025, T026, T027, T028, T029]
---

# WP05: End-to-End Verification

## Objective

Validate the entire simulation engine stack (Geometry -> Physics -> Rendering -> API) using a realistic scenario and formal tests.

## Context

Up to this point, we have built the system in layers. Now we must prove it works as a coherent whole.
We will use a "Pusher Bot" scenario: A simple robot arm (or sliding block) pushing a payload into a target zone. This tests collision, zones, physics stability, and rendering.

## Implementation Guide

### T025: Create "Pusher Bot" Reference Model

**File**: `tests/worker/simulation/cases/pusher_bot.py`

1. Use `build123d`.
2. Model:
   - **Floor**: Static box.
   - **Pusher**: A block with a linear joint (prismatic).
   - **Box**: A dynamic cube payload.
   - **Goal Zone**: A specialized object named `zone_goal_box`.
3. Script: define a function `get_pusher_bot() -> Compound`.

### T026: Implement E2E Test Suite

**File**: `tests/e2e/test_simulation_engine.py`

1. Setup generic E2E test structure.
2. If running inside Docker/CI, ensure MuJoCo and FFmpeg are available.

### T027: Verify "Success" Scenario

**Goal**: The Pusher successfully pushes the box into the goal.

1. Test case: `test_pusher_success()`
   - Load `get_pusher_bot()`.
   - Call `simulate_local()` with a control function that moves the pusher forward.
   - Assert:
     - `outcome == "SUCCESS"`
     - `video_url` is not None.
     - `metrics["total_time"] > 0`.
   - Verify artifacts: Check if the video file is valid (mocked S3 or local check).

### T028: Verify "Fail" Scenario

**Goal**: The Pusher hits a forbidden zone or fails to reach the goal.

1. Test case: `test_pusher_fail()`
   - Variant A: Modify control to push the box AWAY from goal -> Outcome `FAIL` (timeout).
   - Variant B: Add a `zone_forbid_wall` in the path -> Outcome `FAIL` (forbidden zone collision).
2. Assert correct `reason` is returned (e.g. "Collision with zone_forbid_wall").

### T029: Manual Walkthrough & Documentation

**File**: `kitty-specs/003-simulation-engine/quickstart.md` (Update)

1. Document how to run the E2E test.
2. Provide a sample snippet for defining a zone in build123d:

   ```python
   zone = Box(1,1,1, mode=Mode.PRIVATE)
   zone.label = "zone_goal_A"
   assembly.add(zone)
   ```

3. Run the E2E test manually and watch the generated video to confirm visual correctness (lighting, camera angle).

## Validation

- [ ] `pytest tests/e2e/test_simulation_engine.py` passes.
- [ ] Visual inspection of the validation video shows smooth physics (no exploding meshes).

## Risks

- **Simulation Instability**: Box might fall through floor if convex hulls are bad.
  - *Mitigation*: Adjust `margin` or `solimp`/`solref` in MuJoCo options in WP01 if this happens.
- **CI failures**: Rendering might fail in CI.
  - *Mitigation*: Use headless backend.
