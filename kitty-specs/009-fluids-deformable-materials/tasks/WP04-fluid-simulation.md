---
work_package_id: WP04
title: Fluid Simulation (MPM)
lane: "doing"
dependencies: []
base_branch: main
base_commit: 62c53482cecdee76571b526d639b29a6a3949f4f
created_at: '2026-02-15T10:45:15.043718+00:00'
subtasks: [T014, T015, T016, T017]
shell_pid: "76728"
agent: "gemini"
---

# WP04: Fluid Simulation (MPM)

## Objective
Implement MPM fluid simulation in Genesis and evaluate fluid-based objectives (containment and flow rate).

## Context
Fluids are simulated as particles. We need to track these particles against containment zones and gate planes defined in the benchmark.

## Guidance

### T014: Fluid Definition & Spawning
- Support `FluidDefinition` (viscosity, density, surface tension).
- Spawn particles in Genesis using the MPM solver.
- Implement spawning from a volume (box/sphere) defined in the scene.

### T015: Fluid Containment Evaluation
- Count particles inside target `containment_zone` (AABB or sphere).
- Calculate `containment_ratio` (particles_in_zone / total_particles).
- Report in `FluidMetricResult`.

### T016: Flow Rate Evaluation
- Define "gate planes" in the simulation.
- Count particles crossing the plane per second.
- Implement directional counting (positive vs negative flow).

### T017: GPU OOM & WP3 Forward Compatibility
- Detect GPU Out-Of-Memory errors from Genesis/PyTorch.
- Implement auto-retry with 25% fewer particles.
- Add `ELECTRONICS_FLUID_DAMAGE` logic: if fluid particles touch a part with `is_electronics: true`, set failure reason (activation ready for WP3).

## Definition of Done
- [ ] Fluid particles are visible and interact with the environment.
- [ ] Containment metrics are correctly reported in `SimulationResult`.
- [ ] Flow rate is measured and reported.
- [ ] System handles GPU OOM gracefully by reducing fidelity.

## Risks
- High memory consumption for large particle counts.
- Difficulty in tuning MPM parameters for realistic water/oil behaviour.

## Activity Log

- 2026-02-15T10:45:15Z – Gemini – shell_pid=156375 – lane=doing – Assigned agent via workflow command
- 2026-02-15T12:19:27Z – Gemini – shell_pid=156375 – lane=for_review – Ready for review: Implemented MPM fluid simulation, containment/flow-rate metrics, GPU OOM retries, and electronics damage detection.
- 2026-02-15T12:26:25Z – gemini – shell_pid=76728 – lane=doing – Started review via workflow command
