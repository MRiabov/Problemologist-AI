# Benchmark Definition Simplification Refactor

## Scope Summary

- Simplify the benchmark-generation handoff so benchmark planners have one clear place for benchmark objectives/pricing and one clear place for the benchmark CAD plan.
- Keep benchmark objectives, pricing caps, and runtime randomization in `benchmark_definition.yaml`.
- Move the benchmark CAD plan, including the moved object and benchmark-side geometry, into `benchmark_assembly_definition.yaml`.
- Replace the benchmark use of the engineer-style `AssemblyDefinition` with a benchmark-specific assembly model that carries the CAD-plan facts only.
- Keep engineer and electronics planning on `AssemblyDefinition` unchanged.

## Problem Statement

The current benchmark handoff is overloaded and the semantic split is blurry.

- `benchmark_assembly_definition.yaml` is still validated as a full `AssemblyDefinition`.
- That makes benchmark planners satisfy fields they do not need, such as manufactured parts, COTS parts, final assembly structure, and cost totals.
- The benchmark docs already describe benchmark-owned fixtures as read-only context, not manufacturable outputs.
- `benchmark_definition.yaml` already owns the scenario contract and benchmark fixture metadata.
- The benchmark CAD plan, including the moved object, should live in `benchmark_assembly_definition.yaml`; there is no separate `benchmark_parts.yaml` in the repository today.

This overloading makes benchmark completion harder for LLM agents because the benchmark generator has to reason about two different contracts at once:

- the scenario contract and geometry/range contract,
- and a second engineer-style costing contract that does not help benchmark generation.

## Target State

### `benchmark_definition.yaml`

- Owns benchmark objectives, runtime randomization, benchmark-owned fixture metadata, and benchmark/customer caps.
- Remains the single canonical home for `benchmark_parts`.
- Uses `constraints.estimated_solution_cost_usd` and `constraints.estimated_solution_weight_g` for planner-authored estimates.
- Uses `constraints.max_unit_cost` and `constraints.max_weight_g` for benchmark/customer caps.
- Treats benchmark fixture metadata as read-only task context, not engineer manufacturability input.

### `benchmark_assembly_definition.yaml`

- Validates against a new `BenchmarkAssemblyDefinition` model, not `AssemblyDefinition`.
- Carries the benchmark CAD plan: moved object geometry/placement, benchmark-owned fixture geometry, and benchmark-side motion and interaction metadata.
- Supports a small, explicit `parts` list with fields such as `part_id`, `label`, `dofs`, `control`, `motion_limits`, and reviewer-facing notes.
- Does not require `manufactured_parts`, `cots_parts`, `final_assembly`, `totals`, or benchmark/customer cap fields.
- Stays schema-valid even when the benchmark has no moving fixtures.
- Does not imply benchmark fixture geometry must be modeled as engineer-manufacturable assembly geometry.

### `assembly_definition.yaml`

- Remains the engineer/electronics costing and manufacturing contract.
- Continues to use `AssemblyDefinition`.
- Continues to own engineer-owned manufactured parts, COTS parts, final assembly structure, and planner targets.

### Compatibility Policy

- The final state must not silently coerce benchmark handoffs back into `AssemblyDefinition`.
- Any temporary migration shim is acceptable only during rollout and must be removed before the refactor is accepted.
- The benchmark contract should fail closed if a stale benchmark assembly artifact still looks like a full engineer assembly.

## Required File Changes

| Area | Files | Required change |
| -- | -- | -- |
| Shared schemas | `shared/models/schemas.py` | Add `BenchmarkAssemblyDefinition` and the benchmark motion-part model(s); keep `AssemblyDefinition` for engineer planning only; move the benchmark CAD-plan fields, including `moved_object`, out of the benchmark scenario model. |
| Benchmark validation | `worker_heavy/utils/file_validation.py`, `worker_heavy/utils/validation.py`, `worker_heavy/utils/handover.py` | Parse and validate benchmark handoffs with the new benchmark-specific schema; remove benchmark costing assumptions from benchmark assembly validation; keep benchmark motion and attachment policy checks fail-closed. |
| Controller benchmark plumbing | `controller/agent/benchmark/tools.py`, `controller/agent/benchmark_handover_validation.py`, `controller/agent/node_entry_validation.py`, `controller/agent/review_handover.py`, `controller/agent/handover_constants.py`, `controller/api/routes/datasets.py`, `controller/api/tasks.py` | Update file sets, manifest generation, artifact routing, and review gating to use the simplified benchmark assembly contract. |
| Prompted agent nodes | `controller/agent/benchmark/nodes.py`, `controller/agent/nodes/planner.py`, `controller/agent/nodes/electronics_planner.py`, `controller/agent/nodes/plan_reviewer.py`, `controller/agent/nodes/coder.py`, `controller/agent/nodes/execution_reviewer.py`, `controller/agent/nodes/electronics_reviewer.py` | Rewrite prompt text and file expectations so benchmark planners/reviewers reason over benchmark CAD-plan data instead of a full costing assembly. |
| Shared templates | `shared/assets/template_repos/benchmark_generator/*`, `shared/agent_templates/__init__.py`, `controller/agent/initialization.py` | Regenerate benchmark starter files, including `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, and `result.py`, so the benchmark template matches the simplified contract. |
| Seeded artifacts and mock responses | `tests/integration/mock_responses/**/benchmark_*`, `dataset/data/seed/artifacts/**/benchmark_*` | Update or regenerate all benchmark fixture artifacts so seeded workspaces and mock transcripts reflect the new schema. |
| Public docs and architecture | `specs/architecture/CAD-and-other-infra.md`, `specs/architecture/agents/handover-contracts.md`, `specs/architecture/agents/roles.md`, `specs/architecture/agents/tools.md`, `specs/architecture/agents/artifacts-and-filesystem.md`, `specs/architecture/simulation-and-dod.md`, `specs/architecture/evals-architecture.md`, `specs/architecture/application-acceptance-criteria.md`, `docs/backend-reference.md`, `docs/data-models.md`, `specs/integration-tests.md` | Update the public contract so future agents see `benchmark_definition.yaml` as the scenario/pricing file and `benchmark_assembly_definition.yaml` as the benchmark CAD-plan file. |
| Integration tests | `tests/integration/architecture_p0/test_int_008_objectives_validation.py`, `tests/integration/architecture_p0/test_planner_gates.py`, `tests/integration/architecture_p0/test_node_entry_validation.py`, `tests/integration/architecture_p1/test_benchmark_workflow.py` | Replace assertions that benchmark assembly is a full `AssemblyDefinition`; add coverage for the new benchmark-specific model and reject stale benchmark assembly shapes. |

## Implementation Decisions

1. The benchmark fixture list stays in `benchmark_definition.yaml`.
2. The benchmark CAD plan, including the moved object, lives in `benchmark_assembly_definition.yaml`.
3. No new standalone `benchmark_parts.yaml` file is introduced in this phase.
4. Benchmark caps stay in `benchmark_definition.yaml.constraints`.
5. The new benchmark assembly model is benchmark-specific and intentionally smaller than `AssemblyDefinition`.
6. Engineer planning continues to use `AssemblyDefinition` and keeps its existing manufacturability and costing behavior.

## Acceptance Criteria

1. Given a benchmark planner workspace with valid benchmark artifacts, when `submit_plan()` runs, then the benchmark handoff succeeds with objectives/pricing in `benchmark_definition.yaml` and the moved object plus CAD plan in `benchmark_assembly_definition.yaml`, without `manufactured_parts`, `cots_parts`, `final_assembly`, `totals`, or benchmark/customer cap fields in the benchmark assembly file.
2. Given a benchmark definition file, when it is parsed, then `benchmark_parts` remains the canonical benchmark fixture list and the benchmark caps are available from `benchmark_definition.yaml.constraints`.
3. Given a benchmark assembly file, when it is parsed, then it validates against `BenchmarkAssemblyDefinition` and exposes the moved object plus benchmark geometry, motion, and interaction metadata.
4. Given a benchmark handoff with a stale `AssemblyDefinition`-shaped benchmark assembly artifact, when benchmark validation or node-entry preflight runs, then the system fails closed instead of coercing the file into the old engineer schema.
5. Given an engineer planner or electronics planner workspace, when it reads benchmark context, then it still receives the benchmark-owned read-only fixture metadata, but it does not depend on benchmark assembly costing fields.
6. Given the benchmark and engineer integration suites, when they run through `./scripts/run_integration_tests.sh` on the relevant slices, then the updated benchmark and engineer flows pass with the simplified contract.
7. Given the architecture docs and templates, when a new agent reads the benchmark contract, then the docs describe `benchmark_definition.yaml` as the scenario/pricing file and `benchmark_assembly_definition.yaml` as the benchmark CAD-plan file.

## Verification Plan

- Validate the schema split with the narrowest benchmark planning and node-entry tests first.
- Then widen to the benchmark workflow and seeded artifact tests.
- Do not use unit tests as the acceptance gate for this refactor.
- Keep the integration assertions explicit about the final file contract rather than relying on implicit compatibility behavior.

## Risks

- This is a high-churn refactor because the benchmark contract is threaded through schemas, controller plumbing, worker validation, templates, seeded datasets, mock responses, and integration tests.
- The biggest failure mode is leaving a hidden fallback path that still accepts benchmark handoffs as a full `AssemblyDefinition`.
- The second failure mode is updating the schema but forgetting the seeded benchmark artifacts and prompt text, which would leave the agents with mixed contracts.
- The migration only pays off if the old benchmark assembly coupling is removed everywhere that benchmark planners and reviewers touch it.

## Out Of Scope

- Do not rework engineer `AssemblyDefinition` beyond the minimum needed to keep shared schema imports coherent.
- Do not change simulation physics, solver behavior, or benchmark solvability rules except where the benchmark handoff shape forces a validator update.
- Do not introduce a separate `benchmark_parts.yaml` file in this phase.
- Do not weaken fail-closed validation for benchmark geometry, attachment policy, or motion declarations.
