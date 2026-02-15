---
work_package_id: WP07
title: Benchmark Planning & Integration Tests
lane: "doing"
dependencies: []
base_branch: main
base_commit: 6d83d6425a3faf5bd47da49d39daf21ff3340bf4
created_at: '2026-02-15T19:21:55.248649+00:00'
subtasks: [T025, T026, T027]
shell_pid: "438305"
agent: "gemini"
review_status: "has_feedback"
reviewed_by: "MRiabov"
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

## Activity Log

- 2026-02-15T19:21:55Z – gemini – shell_pid=284334 – lane=doing – Assigned agent via workflow command
- 2026-02-15T19:54:55Z – gemini – shell_pid=284334 – lane=for_review – Implemented benchmark planner updates and integration tests for FEM, fluids, and GPU OOM retry. Verified with pytest.
- 2026-02-15T20:48:22Z – Antigravity – shell_pid=284334 – lane=planned – Moved to planned
- 2026-02-15T21:01:57Z – Gemini – shell_pid=410766 – lane=doing – Started implementation via workflow command
- 2026-02-15T21:08:02Z – Gemini – shell_pid=410766 – lane=for_review – Ready for review: implemented benchmark planner updates for Genesis/FEM/Fluids, added integration tests for physics parity, FEM breakage, fluid containment, and GPU OOM retry logic. Verified with pytest.
- 2026-02-15T21:20:08Z – gemini – shell_pid=438305 – lane=doing – Started review via workflow command
