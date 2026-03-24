# TODO List - Benchmark Generator

## Planning

- [ ] Confirm the benchmark stays in the simple rigid-body, gravity-only family.
- [ ] Define the passive static geometry and the sphere input object in `plan.md`.
- [ ] Write objective zones and the build zone so the gravity path is clear.
- [ ] Record the cost and weight envelope for the passive fixture.

## Handoff Artifacts

- [ ] Write `benchmark_definition.yaml` with rigid-body-only scope and explicit bounds.
- [ ] Write `benchmark_assembly_definition.yaml` with a schema-valid passive fixture handoff.
- [ ] Keep any benchmark-side motion out of scope unless it remains passive and reviewable.

## Validation

- [ ] Verify geometry, objective clearance, and runtime jitter robustness.
- [ ] Reject any actuators, FEM, or fluids before submission.
- [ ] Call `submit_plan()` only after the handoff files are internally consistent.
