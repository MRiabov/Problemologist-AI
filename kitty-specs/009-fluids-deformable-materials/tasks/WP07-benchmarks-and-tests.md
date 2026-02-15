---
work_package_id: WP07
title: Benchmark Planning & Integration Tests
lane: planned
dependencies: []
subtasks: [T025, T026, T027]
---

# WP07: Benchmark Planning & Integration Tests

## Objective
Update the benchmark planner to utilize the new physics capabilities and verify the entire system with integration tests.

## Context
We need automated ways to prove that Genesis and MuJoCo are consistent for rigid bodies and that the new FEM/Fluid features work as specified.

## Guidance

### T025: Benchmark Planner Updates
- Update the Planner agent's tools to support generating `objectives.yaml` with:
  - `physics: {backend: genesis, fem_enabled: true}`
  - `fluid_objectives` (containment, flow_rate)
  - `stress_objectives` (max_stress thresholds)

### T026: Physics Parity Tests
- Create `tests/integration/test_physics_parity.py`.
- Run standard rigid-body benchmarks on both MuJoCo and Genesis.
- Compare final positions and joint angles. Tolerance: < 5% difference.

### T027: Feature Integration Tests
- `tests/integration/test_fem_breakage.py`: Verify that exceeding ultimate stress triggers failure.
- `tests/integration/test_fluid_containment.py`: Verify that fluid particles are correctly counted against zones.
- `tests/integration/test_gpu_oom_retry.py`: Mock a GPU OOM and verify the system retries with lower fidelity.

## Definition of Done
- [ ] Planner agent can create complex fluid/stress benchmarks.
- [ ] Physics parity tests pass (SC-004).
- [ ] All feature integration tests pass reliably.

## Risks
- Flaky integration tests due to non-deterministic physics.
- Genesis vs MuJoCo differences exceeding the 5% tolerance.
