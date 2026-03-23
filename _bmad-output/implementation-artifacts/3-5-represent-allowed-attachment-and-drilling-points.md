# Story 3.5: Represent Allowed Attachment and Drilling Points

Status: ready-for-dev

## Story

As a human operator, I want the system to represent allowed attachment and drilling points in the benchmark setup so that CAD and manufacturability review can validate how the design may be connected or modified.

## Acceptance Criteria

1. Given a benchmark with declared attachment or drilling points, when the benchmark package is reviewed, then those points are persisted, visible, and tied to the latest revision.
2. Given missing, conflicting, or unsupported attachment or drilling points, when validation runs, then the system rejects or flags the benchmark with an explicit reason.
3. Given a CAD revision changes attachment or drilling constraints, when the benchmark is reopened, then the latest revision reflects the updated attachment and drilling rules in the review artifacts.
4. Given benchmark-owned fixture metadata declares `attachment_policy`, when the planner or reviewer reads the handoff package, then the policy is treated as read-only benchmark context and is not rewritten into engineer-owned assembly metadata.
5. Given a benchmark fixture allows fastener mounting or drilling, when handoff validation runs, then the engine enforces the whole-part allowlist contract and the declared `drill_policy` numeric limits, including hole count, diameter range, and depth.
6. Given a benchmark fixture has no `attachment_policy`, when planning or validation runs, then the fixture is treated as non-attachable and non-drillable by default.

## Tasks / Subtasks

- [ ] Extend the shared schema contracts for benchmark fixture attachment metadata.
  - [ ] Review `shared/models/schemas.py` to ensure `BenchmarkPartAttachmentPolicy` and `BenchmarkPartDrillPolicy` carry the typed allowlist fields used by the architecture contract (`attachment_methods`, `drill_policy`, and reviewer-facing `notes`).
  - [ ] Keep validation strict: `attachment_methods` must remain limited to `fastener` and `none`, and `drill_policy.allowed=true` must continue to require fastener attachment permission.
  - [ ] Preserve fail-closed behavior for unsupported or missing attachment-policy shapes instead of introducing permissive fallbacks.
- [ ] Tighten benchmark handoff validation around attachment/drilling declarations.
  - [ ] Update `worker_heavy/utils/file_validation.py` so benchmark-side attachment and drilling constraints are validated against the declared `attachment_policy` on the benchmark-owned part.
  - [ ] Ensure missing `attachment_policy` means non-attachable/non-drillable by default.
  - [ ] Keep rejection reasons explicit for forbidden drilling, unsupported attachment methods, invalid hole counts, diameter mismatches, and depth violations.
- [ ] Keep planner and node-entry validation aligned with the benchmark attachment contract.
  - [ ] Update `controller/agent/node_entry_validation.py` and any planner-handoff validation helper that reads benchmark fixtures so the latest revision reflects the declared benchmark-side attach/drill rules.
  - [ ] Ensure planner review artifacts and benchmark package persistence surface the attachment policy on the latest revision rather than silently dropping it.
- [ ] Keep authored templates and prompts aligned with the benchmark contract.
  - [ ] Update `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml` and `shared/assets/template_repos/engineer/assembly_definition.yaml` if the template examples need to show the benchmark-owned attachment/drill policy explicitly.
  - [ ] Update `config/prompts.yaml` only if the current prompt leaves ambiguity about benchmark-owned attachment rules or undeclared drilling being invalid.
- [ ] Extend integration coverage for attachment and drilling points.
  - [ ] Add or refresh coverage in `tests/integration/architecture_p0/test_int_008_objectives_validation.py` for typed attachment-policy validation on benchmark fixtures.
  - [ ] Add or refresh coverage in `tests/integration/architecture_p0/test_planner_gates.py` for reject/flag paths on unsupported drilling and attachment contracts.
  - [ ] Add or refresh end-to-end handoff coverage in `tests/integration/architecture_p0/test_planner_gates.py` or `tests/integration/architecture_p1/test_handover.py` to prove the latest revision carries the benchmark attachment/drill metadata into review artifacts.
  - [ ] Keep assertions HTTP-, schema-, trace-, and artifact-based; do not add unit-only coverage.
- [ ] Verify the relevant integration slices before closing the story.
  - [ ] At minimum cover `INT-008`, `INT-010`, `INT-018`, `INT-023`, `INT-031`, `INT-032`, `INT-033`, and `INT-034` if the implementation touches persistence or handoff review artifacts.

## Dev Notes

- Epic 3, Story 3.5 is the source of truth for the human-facing requirement.
- The benchmark-side contract is whole-part in MVP. Do not invent zone-level or coordinate-level drilling rules in this story.
- `benchmark_definition.yaml` owns benchmark fixture metadata only. Engineer-owned assembly metadata stays in `assembly_definition.yaml`.
- The architecture contract is permissive, not mandatory: allowed benchmark attachment/drilling paths may be used by the engineer, but they are not required if the benchmark can be solved another way.
- If a benchmark fixture exposes no `drill_policy`, drilling is forbidden by default.
- `environment_drill_operations` is the planner-declared runtime drilling record and must remain consistent with the benchmark-side `drill_policy`.
- This story should preserve strict schema validation and explicit error reasons. Do not create a silent fallback from missing benchmark attachment metadata to permissive attachment behavior.

### Project Structure Notes

- Keep benchmark attachment policy in the shared schema and benchmark handoff validation path. Do not create a second attachment-policy representation in prompt text alone.
- Preserve ownership boundaries: benchmark-owned fixture metadata is read-only context, not engineer-owned solution data.
- Keep failure reasons explicit and deterministic so downstream reviewer evidence and evals can distinguish forbidden attachment from allowed attachment.
- Do not relax the drill policy contract into a generic freeform notes field. The typed policy is the machine-enforced contract.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 3.5: Represent Allowed Attachment and Drilling Points]
- [Source: \_bmad-output/planning-artifacts/prd.md, manufacturability, benchmark handoff, and review-quality requirements]
- [Source: specs/desired_architecture.md, architecture index for agents, evaluation gates, and CAD/infrastructure contracts]
- [Source: specs/architecture/primary-system-objectives.md, product-level benchmark and engineering goals]
- [Source: specs/architecture/agents/overview.md, benchmark/engineer workflow split]
- [Source: specs/architecture/agents/roles.md, planner/reviewer responsibilities and handoff artifacts]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark fixture ownership rules, attachment-policy contract, and drill-policy limits]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, latest-revision handoff and reviewer artifact expectations]
- [Source: specs/architecture/agents/tools.md, planner gate and handoff submission rules]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark-owned fixture metadata and attachment/drill policy contract]
- [Source: specs/architecture/simulation-and-dod.md, benchmark attachment and drilling validation rules]
- [Source: specs/architecture/evals-and-gates.md, fail-closed planner/reviewer gates and benchmark handoff validation]
- [Source: specs/architecture/observability.md, trace and reviewer evidence persistence requirements]
- [Source: shared/models/schemas.py, benchmark attachment/drill policy schemas]
- [Source: worker_heavy/utils/file_validation.py, environment drilling and attachment validation]
- [Source: controller/agent/node_entry_validation.py, planner handoff validation and latest-revision gating]
- [Source: shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml, benchmark template contract]
- [Source: shared/assets/template_repos/engineer/assembly_definition.yaml, engineer assembly template contract]
- [Source: config/prompts.yaml, planner wording for benchmark attachment and drilling discipline]
- [Source: tests/integration/architecture_p0/test_int_008_objectives_validation.py, typed attachment-policy validation coverage]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, planner handoff drilling/attachment rejection coverage]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff coverage]
- [Source: specs/integration-tests.md, INT-008, INT-010, INT-018, INT-023, INT-031, INT-032, INT-033, INT-034]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List

- \_bmad-output/implementation-artifacts/3-5-represent-allowed-attachment-and-drilling-points.md
- \_bmad-output/implementation-artifacts/sprint-status.yaml
