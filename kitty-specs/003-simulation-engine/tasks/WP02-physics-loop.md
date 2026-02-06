---
work_package_id: WP02
title: Physics Simulation Loop
lane: "doing"
dependencies: []
base_branch: main
base_commit: a507a4f4fd2aa747191815080ecb718df03878e2
created_at: '2026-02-06T20:15:49.611838+00:00'
subtasks: [T007, T008, T009, T010, T011, T012, T013]
shell_pid: "7341"
agent: "gemini"
---

# WP02: Physics Simulation Loop

## Objective

Implement the engine that loads the MJCF XML and steps through time, detecting critical events (success/fail) and collecting metrics.

## Context

Standard MuJoCo loops run at fixed timesteps (usually 2ms / 500Hz). We need to wrap this in a python class that can be inspected between steps.
The loop must verify "Zones" defined in WP01.

## Implementation Guide

### T007: Implement SimulationLoop Class

**File**: `src/worker/simulation/loop.py`

1. Class `SimulationLoop`.
2. `__init__(self, xml_path: str)`:
   - Load `self.model = mujoco.MjModel.from_xml_path(xml_path)`
   - Create `self.data = mujoco.MjData(self.model)`

### T008: Implement Physics Stepping

**File**: `src/worker/simulation/loop.py`

1. Method `step(self, control_inputs: dict) -> bool`.
2. Apply controls (force/torque/position) to actuators found in `self.data.ctrl`.
3. Call `mujoco.mj_step(self.model, self.data)`.
4. Run for a max duration (e.g. 10 seconds simulation time).

### T009: Implement Forbidden Zone Detection

**Goal**: Fail if any physical object touches a forbidden zone.

1. In `step()`:
   - Iterate loops over `self.data.contact` (array of contact pairs).
   - Check `geom1` and `geom2` IDs.
   - Use `mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, id)` to get names.
   - If name starts with `zone_forbid`: Trigger FAIL.

### T010: Implement Goal Zone Detection

**Goal**: Succeed if Target Object (e.g., named `target_box`) is is inside `zone_goal`.

1. Identifying inside is tricky with simple Sites.
   - **Method A**: Check center of mass distance. `data.site_xpos['zone_goal']` vs `data.body_xpos['target']`. If dist < threshold -> SUCCESS.
   - **Method B** (Robust): Define `zone_goal` as a phantom Geom (sensor). Check for contact between `target` and `zone_goal`.
   - **Selection**: Use **contact-based** if possible (requires phantom geom in WP01), or strict **AABB containment** logic (check if target position is within zone bounds).
   - Implement simpler AABB check: Get zone bounds from startup (geom size), check if target pos is within logic.

1. Define a Pydantic model `SimulationMetrics`.
1. Track:
   - `total_time`: `data.time`.
   - `total_energy`: sum of `ctrl * velocity` or similar rough proxy.
   - `max_velocity`: peak speed of target.
1. Accumulate these per step and return as a Pydantic model.

### T012: Add Validation Hook

**File**: `src/worker/simulation/loop.py` or entry point

1. Ensure that before the simulation starts, `src.workbenches.validate_and_price` is called.
2. If validation fails, the simulation should raise an exception or return a FAILURE result immediately without stepping.

### T013: Add Tests

**File**: `tests/worker/simulation/test_loop.py`

1. Create a dummy MJCF with a falling ball and a floor.
2. Run `SimulationLoop` for 100 steps.
3. Assert Z-height decreases (simulation is running).
4. Assert collision with floor happens eventually.
5. Setup a "Zone" test: Place ball inside a zone, assert detection triggers.

## Validation

- [ ] Tests pass.
- [ ] Physics behaves deterministically (same seed/start = same result).

## Risks

- **High Speed Tunneling**: Fast objects skipping through thin zones.
  - *Mitigation*: Increase simulation freq (set timestep=0.002 or smaller).

## Activity Log

- 2026-02-06T20:15:49Z – gemini – shell_pid=7341 – lane=doing – Assigned agent via workflow command
