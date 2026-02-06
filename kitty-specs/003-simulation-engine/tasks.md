# Work Packages: 003-simulation-engine

**Feature**: 003-simulation-engine
**Status**: Planned

## WP01: Geometry Pipeline & Scene Builder

**Goal**: Implement the pipeline to convert CAD models (`build123d`) into physics-ready MJCF XML assets, including convex decomposition and zone detection.
**Priority**: Critical
**Prerequisites**: None

- [ ] T001: Initialize `src/worker/simulation` package and install dependencies (`mujoco`, `build123d`, `trimesh`, `scipy`). <!-- id: 0 -->
- [ ] T002: Implement `MeshProcessor` in `builder.py` to convert `build123d` objects to STL. <!-- id: 1 -->
- [ ] T003: Implement Convex Decomposition strategy (V-HACD or convex hull) to ensure stable physics. <!-- id: 2 -->
- [ ] T004: Implement `SceneCompiler` in `builder.py` to generate MJCF XML from processed meshes. <!-- id: 3 -->
- [ ] T005: Implement "Zone Logic" to identify `zone_*` objects and map them to MuJoCo sensors/sites. <!-- id: 4 -->
- [ ] T006: Add unit tests for geometry conversion and XML generation. <!-- id: 5 -->

**Implementation Logic**:

1. Setup package structure.
2. Build conversion utilities for `build123d` -> `trimesh` -> `mujoco`.
3. Detect zones by naming convention (`zone_goal`, `zone_forbid`) and inject into MJCF as sites.
4. Validate generated XML with `mujoco` library.

**Estimated Prompt Size**: ~400 lines

## WP02: Physics Simulation Loop

**Goal**: Implement the core physics execution loop, tracking collisions, zone entries, and collecting metrics.
**Priority**: High
**Dependencies**: WP01

- [ ] T007: Implement `SimulationLoop` class in `src/worker/simulation/loop.py`. <!-- id: 6 -->
- [ ] T008: Implement physics stepping loop (500Hz) and control interface. <!-- id: 7 -->
- [ ] T009: Implement collision detection for `zone_forbid_*`. <!-- id: 8 -->
- [ ] T010: Implement goal detection for `zone_goal_*`. <!-- id: 9 -->
- [ ] T011: Implement `MetricCollector` for energy, time, and damage tracking. <!-- id: 10 -->
- [ ] T012: Add tests for physics steps and collision triggers. <!-- id: 11 -->

**Implementation Logic**:

1. Load MJCF from WP01.
2. Run `mj_step` in a loop.
3. Check `d.contact` for collisions with forbidden zones.
4. Check position of target object relative to goal zone sites.
5. Return simulation outcome (SUCCESS/FAIL).

**Estimated Prompt Size**: ~350 lines

## WP03: Rendering & Artifact Storage

**Goal**: Implement video rendering of the simulation and upload artifacts (video, MJCF) to S3/MinIO.
**Priority**: Medium
**Dependencies**: WP02

- [ ] T013: Implement `Renderer` in `src/worker/simulation/renderer.py` using MuJoCo's GLContext.
- [ ] T014: Implement frame capture for video and 24-view image bundle (8 angles x 3 levels).
- [ ] T015: Implement `ffmpeg` encoding for MP4 and tiling/zipping for image bundle.
- [ ] T016: Implement S3 routing logic via `CompositeBackend` for the `/renders/` directory.
- [ ] T017: Update `SimulationLoop` to integrate rendering and ensure files are saved to `/renders/`.
- [ ] T018: Add tests for video generation and 24-view bundle verification.

**Implementation Logic**:

1. Use `mujoco.Renderer` for offscreen rendering.
2. Pipe frames to `ffmpeg-python` process.
3. Upload outputs to MinIO bucket.

**Estimated Prompt Size**: ~350 lines

## WP04: API & Temporal Integration

**Goal**: Expose the simulation capability via a public API and wrap it in Temporal activities for long-running reliability.
**Priority**: Medium
**Dependencies**: WP03

- [ ] T019: Create `src/worker/simulation/temporal/` activities. <!-- id: 18 -->
- [ ] T020: Implement `run_simulation_activity` which wraps the full pipeline. <!-- id: 19 -->
- [ ] T021: Implement `src/worker/utils/simulation.py` with `simulate()` entrypoint. <!-- id: 20 -->
- [ ] T022: Implement `SimulationResult` Pydantic model and return logic. <!-- id: 21 -->
- [ ] T023: Integrate with Temporal Worker startup. <!-- id: 22 -->
- [ ] T024: Add integration tests for the full flow (mocking Temporal if needed). <!-- id: 23 -->

**Implementation Logic**:

1. Define the Temporal Activity.
2. The public `simulate()` function triggers the activity (or executes locally for MVP).
3. Ensure proper error handling and result formatting.

**Estimated Prompt Size**: ~400 lines

## WP05: End-to-End Verification

**Goal**: Validate the entire system with realistic "Pusher Bot" scenario.
**Priority**: Low
**Dependencies**: WP04

- [ ] T025: Create a reference "Pusher Bot" `build123d` model and test script. <!-- id: 24 -->
- [ ] T026: Create an E2E test suite in `tests/e2e/test_simulation.py`. <!-- id: 25 -->
- [ ] T027: Verify "Success" scenario (bot pushes box to goal). <!-- id: 26 -->
- [ ] T028: Verify "Fail" scenario (bot hits forbidden zone). <!-- id: 27 -->
- [ ] T029: detailed manual walkthrough verification. <!-- id: 28 -->

**Implementation Logic**:

1. Run full system tests against the `simulate()` API.
2. Verify S3 artifacts are actually playable.
3. Verify metrics are accurate.

**Estimated Prompt Size**: ~300 lines
