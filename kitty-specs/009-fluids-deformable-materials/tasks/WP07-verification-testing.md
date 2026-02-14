---
work_package_id: "WP07"
title: "Verification & Testing"
lane: "planned"
dependencies: "[]"
subtasks: ["T024", "T025", "T026"]
---

# WP07: Verification & Testing

## Objective
Ensure system reliability and physics accuracy through comprehensive integration tests covering backend parity, breakage detection, and fluid metrics.

## Context
Physics simulations are notoriously difficult to test. We need a solid suite of integration tests that verify our core physical invariants and ensure no regressions in existing features.

## Detailed Guidance

### T024: Backend parity tests
- Write `tests/integration/test_physics_parity.py`.
- Run a set of rigid-body benchmarks (e.g., sliding box, falling ball) on both MuJoCo and Genesis.
- Assert that final positions/velocities are within a 5% tolerance.

### T025: Part breakage tests
- Write `tests/integration/test_fem_breakage.py`.
- Create a synthetic benchmark with a part known to break under a specific load.
- Assert that simulation fails with `PART_BREAKAGE` and returns correct stress values.

### T026: Fluid metric tests
- Write `tests/integration/test_fluid_metrics.py`.
- Create a fluid containment benchmark.
- Verify that spilling water (e.g., by tipping a container) correctly updates the containment ratio and triggers objective failure if below threshold.

## Test Strategy
- Use `pytest` with markers for slow simulations.
- Ensure tests run on both CPU (smoke test) and GPU (if available).
- Mock agent interactions where necessary to test end-to-end flows.

## Definition of Done
- [ ] Backend parity tests pass.
- [ ] Breakage detection tests pass.
- [ ] Fluid containment and flow rate tests pass.
- [ ] All WP2 features are verified in the CI pipeline.

## Risks
- Non-deterministic behavior in Genesis making parity tests flaky.
- Long test execution times on CPU-only machines.
- GPU-only bugs not caught in CPU CI runs.
