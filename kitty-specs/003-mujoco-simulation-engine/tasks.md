# Work Packages: MuJoCo Simulation Engine

**Feature**: 003-mujoco-simulation-engine
**Status**: Planned

## WP01: Project Structure & Mesh Pipeline

**Goal**: Initialize the module and implement the geometry-to-physics asset pipeline.
**Priority**: Critical (Foundation)
**Prerequisites**: None

- [ ] T001: Initialize `src/simulation_engine` package and install dependencies (mujoco, build123d, trimesh).
- [ ] T002: Implement `MeshProcessor` to convert `build123d` objects to temporary STL/GLB files.
- [ ] T003: Implement Convex Decomposition logic (using `trimesh.convex.convex_hull` or `vhacd`).
- [ ] T004: Write unit tests for mesh conversion and hull generation.

**Implementation Logic**:

1. Setup proper package structure.
2. Create `builder.py` with mesh utilities.
3. Use `trimesh` to load generated STLs and compute convex hulls (crucial for stable physics).
4. Save processed meshes to a temporary assets directory.

## WP02: Scene Compiler (MJCF Generator)

**Goal**: Convert the Processed Geometry into a full MuJoCo XML definition.
**Priority**: High
**Prerequisites**: WP01

- [ ] T005: Implement `SceneCompiler` class to generate MJCF XML structure.
- [ ] T006: Implement "Zone Logic" - map `zone_*` named objects to MuJoCo sites/sensors.
- [ ] T007: Implement "Actuator Injection" - parse agent joints and add `<motor>` tags.
- [ ] T008: Verify generated XML is valid by loading it with `mujoco.MjModel.from_xml_string`.

**Implementation Logic**:

1. Iterate over the CAD compound.
2. If name starts with `zone_`, create a sensor/site.
3. If name is `obstacle_`, create a static geom.
4. If part of Agent, create dynamic body with joints.
5. Produce final `scene.xml`.

## WP03: Simulation Runtime Core

**Goal**: Execute the physics loop with agent control and metric tracking.
**Priority**: High
**Prerequisites**: WP02

- [ ] T009: Implement `SimulationLoop` class in `simulation.py`.
- [ ] T010: Implement `AgentInterface` to call user-provided control functions.
- [ ] T011: Implement `MetricCollector` to track energy, time, and collisions.
- [ ] T012: Implement "Termination Conditions" (Goal reached, Forbidden zone, Timeout).

**Implementation Logic**:

1. Load the MJCF model.
2. Run standard `while not done:` loop.
3. At each step, read sensors -> call agent -> apply control -> `mj_step`.
4. Check collisions using `d.contact` and geom IDs.
5. Capture metrics accumulator.

## WP04: Process Isolation & API Service

**Goal**: Wrap the simulation in a safe sandbox and expose it via HTTP.
**Priority**: Medium
**Prerequisites**: WP03

- [ ] T013: Implement `ProcessRunner` in `runner.py` to run `SimulationLoop` in a separate process.
- [ ] T014: Implement timeout and exception safety in `ProcessRunner`.
- [ ] T015: Create `main.py` with FastAPI app and `POST /simulate` endpoint.
- [ ] T016: Define API request/response models (Pydantic).

**Implementation Logic**:

1. Use `multiprocessing.Process` (or `concurrent.futures.ProcessPoolExecutor`) to run the sim.
2. Pass data via Queue or Pipe.
3. FastAPI handles usage validation and async awaiting of the result.

## WP05: End-to-End Validation & Reporting

**Goal**: Verify the entire system with realistic test cases.
**Priority**: Low
**Prerequisites**: WP04

- [ ] T017: Create a reference "Pusher Bot" `build123d` model + control script.
- [ ] T018: Write E2E test: Submit Pusher Bot -> Waiting for Success Report.
- [ ] T019: Implement optionally returning replay artifacts (video/MJCF).
