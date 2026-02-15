---
work_package_id: "WP01"
title: "Simulation Foundation & Backend Abstraction"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005"]
---

# WP01: Simulation Foundation & Backend Abstraction

## Objective
Introduce the `PhysicsBackend` interface and refactor the system to support multiple simulators (MuJoCo and Genesis). This foundation is required for all subsequent WP2 work.

## Context
Currently, the worker's simulation logic is tightly coupled with MuJoCo. We need an abstraction layer that allows the controller to specify the backend and ensures both engines can be used interchangeably for rigid-body tasks.

## Guidance

### T001: Implement `PhysicsBackend` ABC
- Define `shared/simulation/backends.py`.
- Methods: `load_scene(scene_data)`, `step(dt)`, `get_state()`, `render()`, `get_stress_summaries()`, `get_fluid_metrics()`.
- Use Pydantic models for data exchange.

### T002: Refactor MuJoCo to `MuJoCoBackend`
- Extract existing logic from `worker/simulation/simulator.py` (or similar) into `worker/simulation/mujoco_backend.py`.
- Ensure it implements the `PhysicsBackend` interface.
- Keep performance equivalent to the current implementation.

### T003: `GenesisBackend` Skeleton
- Create `worker/simulation/genesis_backend.py`.
- Implement interface methods with `NotImplementedError` or basic rigid-body stubs.
- Add genesis as a dependency in `pyproject.toml` (if not already present).

### T004: Update Schemas
- Add `SimulatorBackendType` enum (`mujoco`, `genesis`) to `shared/simulation/schemas.py`.
- Update `SimulationRequest` to include `backend: SimulatorBackendType`.
- Update `SimulationResult` to include `failure_reason` extensions: `PART_BREAKAGE`, `ELECTRONICS_FLUID_DAMAGE`, `PHYSICS_INSTABILITY`.

### T005: Update Controller Loop
- In `controller/simulation/loop.py`, read the requested backend from the benchmark/request.
- Instantiate the appropriate backend on the worker side (via the worker's internal routing).
- Validate that fluids are NOT requested if using MuJoCo.

## Definition of Done
- [ ] `PhysicsBackend` interface is defined and used by the controller.
- [ ] MuJoCo still works perfectly for all existing benchmarks.
- [ ] Genesis can be selected via configuration (even if it just fails/stubs).
- [ ] Pydantic schemas include new fields and failure reasons.

## Risks
- Breaking existing MuJoCo simulations during refactoring.
- Performance overhead from the abstraction layer.
