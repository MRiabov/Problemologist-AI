---
work_package_id: "WP03"
title: "Genesis FEM & Rigid Simulation"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T009", "T010", "T011", "T012"]
---

# WP03: Genesis FEM & Rigid Simulation

## Objective
Implement the core physics simulation in Genesis, including rigid bodies, FEM soft bodies, stress monitoring, and breakage detection.

## Context
With the backend abstraction and asset pipeline ready, we now implement the actual simulation logic in Genesis. This is where we move from rigid-only dynamics to realistic engineering simulations where parts can bend and break.

## Detailed Guidance

### T009: Rigid-body Genesis support
- Complete the `GenesisBackend.load_scene` method for rigid bodies.
- Map `build123d` components (as .obj) to `gs.morphs.Mesh`.
- Ensure mass, friction, and restitution are correctly transferred from the model.
- Verify parity with MuJoCo for basic rigid-body benchmarks.

### T010: FEM support in Genesis
- Implement loading of `.msh` files using `gs.morphs.SoftMesh`.
- Map material FEM fields (`youngs_modulus_pa`, `poissons_ratio`) to Genesis material properties.
- Support `material_class`:
  - `rigid`: Use linear FEM.
  - `soft` / `elastomer`: Use hyperelastic (Neo-Hookean) models.
- Handle gravity and contact forces for soft bodies.

### T011: Stress computation and reporting
- In the simulation loop, query the stress field from Genesis for all FEM parts.
- Compute von Mises stress for each element.
- Generate `StressSummary` periodically (e.g., every 50 steps): `max_von_mises_pa`, `safety_factor`, `utilization_pct`.

### T012: Breakage detection logic
- Implement a check in the simulation loop: `if max_von_mises > ultimate_stress_pa: abort`.
- When breakage occurs, capture the failure state (part label, step, location).
- Set `failure_reason: PART_BREAKAGE` in the `SimulationResult`.

## Test Strategy
- **Rigid Parity**: Run a rigid-body "push" benchmark on both backends.
- **FEM Breakage**: Create a benchmark with a thin beam and a heavy load. Verify it fails with `PART_BREAKAGE`.
- **Stress Accuracy**: Compare calculated stress on a simple cantilever beam against analytical formulas (e.g., Euler-Bernoulli).

## Definition of Done
- [ ] Genesis simulates rigid bodies correctly.
- [ ] Manufactured parts deform when FEM is enabled.
- [ ] `PART_BREAKAGE` is correctly detected and reported.
- [ ] Stress summaries are included in the final simulation result.

## Risks
- Numerical instabilities in Genesis (NaN values).
- Performance overhead of high-resolution FEM meshes.
- Inaccurate material property mapping causing unrealistic behavior.
