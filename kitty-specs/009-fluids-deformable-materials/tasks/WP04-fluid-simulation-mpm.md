---
work_package_id: "WP04"
title: "Fluid Simulation (MPM)"
lane: "planned"
dependencies: "[]"
subtasks: ["T013", "T014", "T015"]
---

# WP04: Fluid Simulation (MPM)

## Objective
Implement MPM fluid simulation in Genesis and add objective evaluation for fluid containment and flow rate.

## Context
Fluids are simulated as particles using the Material Point Method (MPM). This allows for complex interactions between liquids and rigid/deformable bodies. We need to define fluids and evaluate if they stay within target zones or flow through designated gates.

## Detailed Guidance

### T013: MPM Fluid support in Genesis
- Implement loading of fluid volumes in `GenesisBackend`.
- Support primitive shapes (box, cylinder) for initial fluid volumes.
- Map `fluids` section from `objectives.yaml` to Genesis MPM materials.
- Set viscosity, density, and surface tension.

### T014: `fluid_containment` metric
- Implement particle-in-zone counting logic.
- Compare particle count against the `containment_zone` (AABB) defined in `objectives.yaml`.
- Calculate ratio $P_{in} / P_{total}$ and verify against threshold.
- Populate `FluidMetricResult` in the simulation result.

### T015: `flow_rate` metric
- Implement particle-gate-crossing logic.
- Define a plane gate and count particles crossing it per second.
- Convert particle count to volumetric flow rate ($L/s$).
- Populate `FluidMetricResult` with measured vs target flow rate.

## Test Strategy
- **Containment Test**: Fill a box with water, shake it (rigid body movement), and verify the containment metric reflects spills.
- **Flow Test**: Open a "valve" (remove a rigid barrier) and verify the flow rate metric captures the particle flux.
- **GPU OOM**: Test with a high particle count to ensure the OOM retry logic (from spec) works.

## Definition of Done
- [ ] Fluids appear and interact in Genesis simulations.
- [ ] `fluid_containment` objectives are correctly evaluated.
- [ ] `flow_rate` objectives are correctly evaluated.
- [ ] Fluid metrics are included in `SimulationResult`.

## Risks
- High VRAM consumption of MPM particles.
- Surface tension artifacts causing "leaky" containers.
- Metric inaccuracy due to particle discretization.
