# Story 6.1: Generate Simple Rigid-Body Benchmarks

Status: in-progress

## Story

As a human operator, I want benchmark generator agents to produce simple rigid-body benchmark candidates so that the benchmark family stays within rigid-body mechanics with gravity enabled.

## Acceptance Criteria

1. Given a simple rigid-body benchmark seed or prompt, when generation runs, then the candidate remains within the simple rigid-body, gravity-enabled scope and does not introduce actuators, FEM, or fluids.
2. Given an unsupported mechanism or modality, when generation runs, then the candidate is rejected rather than silently adapted.
3. Given a generated benchmark candidate, when it is reviewed, then its objective, zones, and motion assumptions are explicit enough for a human operator to use without cleanup.

## Tasks / Subtasks

- [ ] Tighten the benchmark-generator prompt and starter templates so the simple rigid-body, gravity-only family is explicit in the planner output shape and examples.
  - [ ] Keep the existing benchmark planner file contract intact: `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml` at the workspace root.
  - [ ] Make the template language favor passive gravity-driven puzzle geometry and reject any accidental drift toward actuators, FEM, or fluids.
  - [ ] Make the generated candidate descriptions explicit enough for a human operator to use without cleanup and unambiguous enough for downstream engineering workflows.
- [x] Reuse the existing fail-closed benchmark validation and handoff gates to keep unsupported modalities out of the simple rigid-body family.
  - [x] Preserve the current benchmark planner canonicalization and cross-contract checks instead of introducing a parallel validator path.
  - [x] If a new rejection case is needed, make it explicit and deterministic rather than silently converting the candidate into another benchmark family.
- [x] Refresh the benchmark-planner integration fixtures for a representative gravity-only benchmark and one negative unsupported-modality case.
  - [x] Keep the canonical benchmark planner submit-plan trace covered by a simple rigid-body example.
  - [x] Add or update the bad-path fixture so the planner or reviewer rejects unsupported motion assumptions cleanly.
- [x] Extend integration coverage for the simple rigid-body benchmark family and its reviewer gate.
  - [x] Assert that the benchmark definition remains within rigid-body scope and that the generated objective/zone/motion assumptions are explicit.
  - [x] Assert that unsupported mechanisms fail closed before the benchmark is accepted.
  - [x] Keep render-evidence coverage on the plan-review path when images exist.
- [ ] Run the benchmark-generation integration slices that cover planner submission, benchmark-definition validation, and benchmark workflow persistence before closing the story.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 6 is the Phase 1 benchmark-generator family for simple rigid-body mechanics with gravity enabled. Keep it passive, bounded, and reproducible.
  - The benchmark generator is a split workflow: Benchmark Planner -> Benchmark Plan Reviewer -> Benchmark Coder -> Benchmark Reviewer. This story should stay inside that graph and not invent a new benchmark family or a second planner/reviewer pair.
  - The existing benchmark planner gate already canonicalizes `constraints.estimated_solution_cost_usd` and `constraints.estimated_solution_weight_g` into benchmark caps, then validates the full handoff through `submit_plan()`. Reuse that path.
  - `worker_heavy/utils/file_validation.py` already rejects template placeholders, invalid `moved_object.material_id`, MuJoCo+fluids combinations, and invalid benchmark cross-contracts. Do not weaken those guards.
  - `worker_heavy/utils/handover.py` and `controller/agent/benchmark/tools.py` already persist the benchmark plan-review manifest and enforce the planner submission contract. Keep the fail-closed behavior and manifest naming intact.
  - The benchmark plan reviewer must inspect render media with `inspect_media(...)` when renders exist. File listing or text-only references are not sufficient visual review.
  - `config/prompts.yaml` is the source of truth for the benchmark planner and benchmark plan reviewer instructions. Keep any prompt edits aligned with the actual template files and validation logic.
  - The benchmark family should remain within the Phase 1 rigid-body gate from the product docs: no actuators, FEM, or fluids in this story.
  - The current planner prompt already expects `moved_object.start_position` at the top level, unique benchmark part IDs/labels, and a schema-valid full `benchmark_assembly_definition.yaml`. Preserve those contracts.
- Source tree components to touch:
  - `config/prompts.yaml`
  - `controller/agent/benchmark/nodes.py`
  - `controller/agent/benchmark/tools.py`
  - `shared/assets/template_repos/benchmark_generator/plan.md`
  - `shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml`
  - `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml`
  - `worker_heavy/utils/file_validation.py`
  - `worker_heavy/utils/handover.py`
  - `tests/integration/architecture_p0/test_planner_gates.py`
  - `tests/integration/architecture_p0/test_int_008_objectives_validation.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/mock_responses/INT-114.yaml`
- Testing standards summary:
  - Use integration tests only; do not add unit-only coverage for this story.
  - Assert against HTTP responses, persisted artifacts, reviewer manifests, and emitted events.
  - Include at least one expected-fail assertion for unsupported modality or ambiguous motion assumptions.
  - If render images exist for the current revision, the benchmark plan reviewer path must still use `inspect_media(...)`; file listing is not visual inspection.

### Project Structure Notes

- Keep the benchmark-generator family on the existing root-level handoff files. Do not introduce alternate story-specific file locations or a second template pack.
- Reuse the current benchmark planner node and validation path; this story is about making the gravity-only family explicit and reliable, not replacing the runtime contract.
- If you need a representative seed or transcript, prefer updating the existing benchmark planner integration fixtures rather than adding a bespoke harness.
- Because this is the first story in Epic 6, sprint tracking should move `epic-6` from `backlog` to `in-progress`.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 6: Gravity: Benchmarks and Story 6.1]
- [Source: \_bmad-output/planning-artifacts/prd.md, Phase 1 rigid-body mechanics scope, benchmark generation success criteria, and MVP feature set]
- [Source: \_bmad-output/planning-artifacts/architecture.md, Phase 1 rigid-body gate and benchmark-generation scope]
- [Source: specs/business-usecase.md, Phase 1 bounded rigid-body mechanics gate]
- [Source: specs/architecture/agents/overview.md, benchmark generator workflow split]
- [Source: specs/architecture/agents/roles.md, benchmark plan reviewer responsibilities and output expectations]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark handoff contract and manifest naming]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, benchmark planner read/write surfaces and planner submission gate]
- \[Source: specs/architecture/agents/tools.md, benchmark planner tools and `submit_plan()` canonicalization\]
- [Source: specs/architecture/distributed-execution.md, worker boundaries and benchmark validation path]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark-owned fixture metadata and read-only context boundaries]
- [Source: specs/architecture/evals-and-gates.md, benchmark-generator eval targets and fail-closed policies]
- [Source: specs/architecture/simulation-and-dod.md, rigid-body, gravity, and benchmark-fixture motion assumptions]
- [Source: specs/architecture/observability.md, lineage, review, and media-inspection events]
- [Source: specs/integration-tests.md, INT-008, INT-024, INT-114, and INT-188]
- [Source: config/prompts.yaml, benchmark planner and benchmark plan reviewer prompt contracts]
- [Source: shared/assets/template_repos/benchmark_generator/plan.md, benchmark planning scaffold]
- [Source: shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml, benchmark definition scaffold]
- [Source: shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml, benchmark handoff scaffold]
- [Source: controller/agent/benchmark/nodes.py, benchmark planner / reviewer node behavior and normalization]
- [Source: controller/agent/benchmark/tools.py, benchmark submit-plan canonicalization and fail-closed validation]
- [Source: worker_heavy/utils/file_validation.py, benchmark definition and cross-contract validation gates]
- [Source: worker_heavy/utils/handover.py, benchmark handoff submission and review manifest persistence]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, benchmark planner submit-plan trace and validation coverage]
- [Source: tests/integration/architecture_p0/test_int_008_objectives_validation.py, benchmark-definition validation coverage]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark workflow / render / review artifact coverage]
- [Source: tests/integration/mock_responses/INT-114.yaml, benchmark planner submit-plan transcript fixture]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

- Updated the benchmark-generator starter templates and integration fixtures to a passive gravity-only rigid-body example.
- Reverted the prompt-layer experiment after feedback to avoid overfitting the runtime prompts.
- Performed local YAML schema validation and Python syntax compilation on the touched files.
- Did not run the integration slices because the user explicitly asked not to run integration tests.

### Completion Notes List

- Prompt-layer edits were reverted per user feedback; the remaining work is carried by the starter templates, mock fixtures, and integration assertions.
- Updated benchmark-generator starter templates to a passive rigid-body gravity-only example.
- Refreshed INT-114 happy-path fixtures and INT-202 unsupported-motion negative fixtures.
- Extended integration assertions to prove the generated benchmark stays within the rigid-body gravity-only family.
- Local YAML parsing and Python compilation passed for the touched files.
- Integration slices were not executed because the user requested that the integration suite not be run.

### File List

- `shared/assets/template_repos/benchmark_generator/plan.md`
- `shared/assets/template_repos/benchmark_generator/todo.md`
- `shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml`
- `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml`
- `tests/integration/architecture_p0/test_planner_gates.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/mock_responses/INT-114.yaml`
- `tests/integration/mock_responses/INT-114/benchmark_planner/entry_01/01__plan.md`
- `tests/integration/mock_responses/INT-114/benchmark_planner/entry_01/02__todo.md`
- `tests/integration/mock_responses/INT-114/benchmark_planner/entry_01/03__benchmark_assembly_definition.yaml`
- `tests/integration/mock_responses/INT-114/benchmark_planner/entry_01/04__benchmark_definition.yaml`
- `tests/integration/mock_responses/INT-202.yaml`
- `tests/integration/mock_responses/INT-202/benchmark_planner/entry_01/01__plan.md`

### Change Log

- 2026-03-24: Refreshed the benchmark-generator starter templates and mock planner fixtures for a passive gravity-only rigid-body example.
- 2026-03-24: Updated integration assertions to check that INT-114 stays within the rigid-body gravity-only family and that unsupported motion assumptions fail closed in INT-202.
- 2026-03-24: Reverted prompt-layer edits after review feedback to avoid overfitting the benchmark prompts.

### Status

in-progress
