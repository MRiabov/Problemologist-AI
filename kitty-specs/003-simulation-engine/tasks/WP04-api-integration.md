---
work_package_id: WP04
title: API & Temporal Integration
lane: planned
dependencies: []
subtasks: [T020, T021, T022, T023, T024, T025]
---

# WP04: API & Temporal Integration

## Objective

Expose the simulation engine via a high-level API and wrap it in Temporal Activities to ensure durability and handle long-running jobs (e.g., complex 2-minute simulations).

## Context

Worker nodes interact with the rest of the system via Temporal. The Controller triggers a workflow, which calls the Worker's activity.
The simulation itself should be "fire and forget" from the physics loop perspective, but strongly managed by Temporal.

## Implementation Guide

### T020: Create Temporal Activities Module

**File**: `src/worker/simulation/temporal/activities.py`

1. Define `@activity.defn`.
2. Wrapper for `simulate` logic.

### T021: Implement run_simulation_activity

**File**: `src/worker/simulation/temporal/activities.py`

1. Function `run_simulation(payload: SimulationRequest) -> SimulationResult`.
2. Logic:
   - Prepare execution environment (using **deepagents** `SandboxFilesystemBackend`).
   - Call `validate_and_price(component)` as the first step. Fail if invalid.
   - Write `build123d` code/files to disk.
   - Call `SimulationLoop` (via utils).
   - Clean up temp files.
   - Return result.
3. **Heartbeating**: Since simulation is CPU heavy, we should heartbeat every few seconds inside the simulation loop if possible.
   - Pass a "heartbeat callback" to `SimulationLoop` from the activity.

### T022: Implement Public API Utils

**File**: `src/worker/utils/simulation.py`

1. Defines the simplified entry point for other modules in the Worker (e.g. if we want to run without Temporal for debugging).
2. `def simulate(component: Compound, render: bool = False) -> SimulationResult`.
   - Orchestrates Validation -> WP01 (Builder) -> WP02 (Loop) -> WP03 (Render, if requested).

### T023: Implement Data Models

**File**: `src/common/models/simulation.py` (or similar shared path)

1. `SimulationResult(BaseModel)`:
   - `outcome: Literal["SUCCESS", "FAIL"]`
   - `reason: str`
   - `video_url: Optional[str]`
   - `metrics: SimulationMetrics` (Pydantic model from WP02)
   - `artifact_urls: dict` (e.g. `{"mjcf": "..."}`)

### T024: Integrate with Worker Startup

**File**: `src/worker/main.py` (or wherever temporal worker starts)

1. Register `run_simulation` activity with the Temporal Worker.
2. Ensure MJCF compiler dependencies and `SandboxFilesystemBackend` are loaded/ready.

### T025: Integration Tests

**File**: `tests/worker/simulation/test_integration.py`

1. Test `simulate` fully:
   - Input: Simple Box compound.
   - Action: Run generic sim with `render=True`.
   - Assert: Returns `SimulationResult` with `outcome` and `video_url`.
2. Test Temporal Activity (Mocked):
   - Use `temporalio.testing.ActivityEnvironment`.
   - valid inputs -> succeed.
   - exception (e.g. bad geometry) -> fail gracefully (capture error in `reason` vs crashing activity).

## Validation

- [ ] `simulate_local` works end-to-end.
- [ ] Temporal activity test passes.
- [ ] Heartbeating logic is wired up (optional but good).

## Risks

- **Pickling Errors**: Passing complex `build123d` objects to Temporal activities often fails serialization.
  - *Mitigation*: Pass **Source Code** or **Serialized Geometry** (STEP/STL bytes) to the activity, NOT Python objects.
  - **Decision**: The `SimulationRequest` should probably take a *reference* (S3 URI of CAD file) or raw bytes, or simple parameters. If passing source code, execute it inside the activity.
