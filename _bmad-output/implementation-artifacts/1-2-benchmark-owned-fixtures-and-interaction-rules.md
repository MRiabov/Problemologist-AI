# Story 1.2: Benchmark-Owned Fixtures and Interaction Rules

Status: ready-for-dev

## Story

As a human operator, I want to declare benchmark-owned fixtures with explicit interaction permissions so that benchmark context stays read-only for an engineer solving the benchmark unless interaction is explicitly allowed.

## Acceptance Criteria

1. Given benchmark-owned fixtures, when they are declared, then they remain read-only for the engineer by default.
1. Given a fixture marked interactable, when the benchmark is validated and handed off, then the interaction surface is explicit and machine-readable in the persisted benchmark artifacts via the benchmark-owned permission marker.
1. Given a benchmark-owned fixture declaration without explicit interaction permission, when an engineer-facing interaction is implied or attempted, then the handoff fails closed with a deterministic reason instead of inferring permission.
1. Given benchmark-owned fixture declarations in the benchmark seed templates, when they round-trip through YAML serialization and reviewer handoff, then the explicit permission data remains visible in the engineer-read benchmark package.

## Tasks / Subtasks

- [ ] Extend the benchmark fixture schema in `shared/models/schemas.py` so benchmark-owned fixtures can declare explicit interaction permission, including the `allows_engineer_interaction` marker used in the architecture docs, without weakening the current read-only-by-default contract.
  - [ ] Keep the schema strict: default read-only, no silent fallback, no inferred permission from `fixed`, `material_id`, or `cots_id`.
  - [ ] Preserve the existing `attachment_policy` and `drill_policy` semantics; do not use this story to relax fastener/drilling gates.
- [ ] Update the benchmark generator template artifacts in `shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml` and `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml` so the explicit `allows_engineer_interaction` permission is represented in the generated benchmark package where needed.
  - [ ] Keep the benchmark-owned fixture declarations minimal and read-only unless the benchmark intentionally marks a surface interactable.
  - [ ] Make sure the template comments/examples match the real schema fields so prompt-generated seeds do not drift.
- [ ] Thread the permission through benchmark handoff validation and read-only engineer intake in `worker_heavy/utils/file_validation.py` and `worker_heavy/utils/handover.py`.
  - [ ] Fail closed when a benchmark-owned fixture is treated as interactable without the explicit permission marker.
  - [ ] Do not move benchmark-owned metadata into engineer-owned `assembly_definition.yaml`; keep ownership boundaries intact.
- [ ] Extend integration coverage in `tests/integration/architecture_p0/test_int_008_objectives_validation.py`, `tests/integration/architecture_p0/test_planner_gates.py`, and `tests/integration/architecture_p1/test_benchmark_workflow.py`.
  - [ ] Prove that default benchmark fixtures remain read-only.
  - [ ] Prove that an explicit interactable fixture round-trips through the benchmark handoff artifacts.
  - [ ] Prove that missing or invalid interaction metadata fails closed before the engineer consumes the benchmark package.
- [ ] Run the integration slices that cover benchmark-definition validation and benchmark workflow before marking the story complete.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.2 is the source of truth for the human-facing requirement.
  - Benchmark-owned fixtures are read-only task fixtures by default; permission must be explicit, machine-readable, and named consistently with the architecture contract (`allows_engineer_interaction`).
  - `benchmark_assembly_definition.yaml` is read-only benchmark context for engineering, not an engineer-owned costing artifact.
  - Benchmark-owned fixture metadata must stay distinct from engineer-owned `assembly_definition.yaml` and from runtime CAD metadata.
  - Existing attachment and drilling validation already fails closed; this story should add explicit interaction permission, not broaden the set of allowed interactions implicitly.
  - If the benchmark fixture is interactable, the permission must be visible in the artifact that the engineer receives, not only in prose or planner memory.
- Source tree components to touch:
  - `shared/models/schemas.py`
  - `shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml`
  - `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml`
  - `worker_heavy/utils/file_validation.py`
  - `worker_heavy/utils/handover.py`
  - `tests/integration/architecture_p0/test_int_008_objectives_validation.py`
  - `tests/integration/architecture_p0/test_planner_gates.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
- Testing standards summary:
  - Use integration tests only; do not add unit-only coverage.
  - Assert against HTTP responses, persisted artifacts, and review/hand-off outputs.
  - If render evidence becomes relevant in a path you touch, the reviewer path must use `inspect_media(...)`; file listing is not image review.

### Project Structure Notes

- Keep benchmark-owned interaction metadata in the shared schema layer, not as ad hoc dicts in worker code.
- Keep the benchmark template repository as the source of seed examples; do not duplicate the contract in multiple runtime folders.
- Preserve the separation between benchmark-owned read-only context and engineer-owned solution artifacts. This story is about making the boundary explicit, not about moving ownership.
- Recent git history is docs/formatting-only, so there is no fresh implementation pattern to copy from the last few commits.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 1.2: Benchmark-Owned Fixtures and Interaction Rules]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark-owned fixture read-only contract and explicit interaction-permission rules]
- \[Source: specs/architecture/agents/roles.md, Engineering Planner intake must read `benchmark_assembly_definition.yaml` as read-only context\]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark-owned fixture metadata, read-only benchmark context, and template/render contracts]
- [Source: specs/architecture/simulation-and-dod.md, benchmark-only motion exception and benchmark-side DOF guidance]
- \[Source: shared/models/schemas.py, `BenchmarkPartMetadata`, `BenchmarkPartAttachmentPolicy`, and `BenchmarkPartDrillPolicy`\]
- \[Source: worker_heavy/utils/file_validation.py, `validate_environment_attachment_contract` and benchmark-definition validation\]
- [Source: worker_heavy/utils/handover.py, benchmark handoff validation and stage-specific artifact checks]
- [Source: shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml, benchmark definition seed template]
- [Source: shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml, benchmark handoff seed template]
- [Source: tests/integration/architecture_p0/test_int_008_objectives_validation.py, benchmark-definition schema/semantic validation coverage]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, benchmark handoff and attachment/drilling gate coverage]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark generation workflow and artifact persistence coverage]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
