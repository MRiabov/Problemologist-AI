---
work_package_id: "WP01"
title: "Simulation Foundation & Abstraction"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005"]
---

# WP01: Simulation Foundation & Abstraction

## Objective
Establish the `PhysicsBackend` protocol to allow swapping between MuJoCo and Genesis, and update the shared schemas and material configurations to support FEM and fluids.

## Context
Currently, the system is hardcoded to use MuJoCo. We need to introduce an abstraction layer so we can support Genesis (which handles FEM and fluids) while keeping MuJoCo as a fast, rigid-only alternative. We also need to prepare the data models for the new physical properties.

## Detailed Guidance

### T001: Implement `PhysicsBackend` protocol
- Create `shared/simulation/backends.py`.
- Define a `typing.Protocol` named `PhysicsBackend`.
- Required methods:
  - `load_scene(scene: SimulationScene) -> None`
  - `step(dt: float) -> StepResult`
  - `get_body_state(body_id: str) -> BodyState`
  - `get_stress_field(body_id: str) -> StressField | None`
  - `get_particle_positions() -> np.ndarray | None`
  - `render_camera(camera_name: str, width: int, height: int) -> np.ndarray`

### T002: Refactor MuJoCo implementation
- Move the existing MuJoCo logic from the worker's simulation loop into a new class `MujocoBackend` in `worker/simulation/mujoco_backend.py`.
- Ensure it implements the `PhysicsBackend` protocol.
- `get_stress_field` and `get_particle_positions` should return `None` for MuJoCo.

### T003: GenesisBackend skeleton
- Create `worker/simulation/genesis_backend.py` with a skeleton `GenesisBackend` class.
- Implement `load_scene` with a basic entity mapping (rigid bodies first).
- Use `import genesis as gs` (ensure the environment has it or provide instructions for installation).

### T004: Update `manufacturing_config.yaml` and schema
- Update the Pydantic models for materials in `shared/models/manufacturing.py` (or similar).
- Add fields: `youngs_modulus_pa`, `poissons_ratio`, `yield_stress_pa`, `ultimate_stress_pa`, `elongation_at_break`, `material_class`.
- Update `config/manufacturing_config.yaml` with default values for common materials (Aluminum 6061, ABS plastic, Silicone rubber).

### T005: Update `SimulationResult` and shared schemas
- Update `SimulationResult` in `shared/models/simulation.py`.
- Add `stress_summaries: list[StressSummary] = []` and `fluid_metrics: list[FluidMetricResult] = []`.
- Extend `failure_reason` enum with `PART_BREAKAGE`, `FLUID_OBJECTIVE_FAILED`, etc.

## Test Strategy
- Unit tests for the new schema validation.
- Smoke test: Run an existing rigid-body benchmark with `backend: mujoco` and verify it still works.

## Definition of Done
- [ ] `PhysicsBackend` protocol defined and correctly used by worker.
- [ ] MuJoCo logic isolated and verified.
- [ ] Genesis skeleton exists and loads without errors.
- [ ] Materials configuration contains all necessary FEM fields.
- [ ] `SimulationResult` supports new metric types.

## Risks
- Regressions in existing rigid-body simulations.
- Schema mismatches between controller and worker.
