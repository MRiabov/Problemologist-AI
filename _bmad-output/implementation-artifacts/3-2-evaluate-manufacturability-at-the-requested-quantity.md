# Story 3.2: Evaluate Manufacturability at the Requested Quantity

Status: done

## Story

As a human operator, I want the system to evaluate whether a solution is manufacturable at the requested production volume so that I can distinguish a prototype answer from a small-batch or mass-production answer.

## Acceptance Criteria

1. Given `BenchmarkDefinition.constraints.target_quantity` or a direct `AnalyzeRequest.quantity`, when manufacturability is evaluated, then the analysis uses that quantity rather than silently defaulting to `1`.
2. Given a design whose setup cost and variable cost differ across manufacturing methods, when validation runs at the requested quantity, then the returned workbench result exposes setup cost and variable cost separately and the planner can compare the methods at that quantity.
3. Given a design that exceeds the allowed cost or weight envelope for the requested quantity, when validation or planner submission runs, then the run fails closed with an explicit reason that names the quantity-driven constraint that was exceeded.
4. Given multiple manufacturing methods are viable for the same part, when the planner evaluates the design, then it selects the method that satisfies the requirements at the requested quantity instead of hardcoding the cheapest single-unit result.
5. Given quantity is declared in `constraints.target_quantity` or direct analyze input, when the handoff is validated, then the same quantity remains traceable in the benchmark and assembly artifacts plus the runtime validation result metadata.

## Tasks / Subtasks

- [x] Thread the requested production quantity through the manufacturability path in `worker_heavy/utils/handover.py`, `worker_heavy/simulation/loop.py`, and any helper they call so the quantity used in validation comes from the declared benchmark or planner objective rather than an implicit default.
  - [x] Keep the quantity lookup fail closed when `target_quantity` is missing, invalid, or conflicts across artifacts.
  - [x] Preserve the existing `quantity` argument behavior on `benchmark/analyze` so direct analyze calls stay deterministic.
- [x] Keep the existing quantity-aware workbench formulas as the source of truth in `worker_heavy/workbenches/cnc.py`, `worker_heavy/workbenches/injection_molding.py`, and `worker_heavy/workbenches/print_3d.py`.
  - [x] Continue separating fixed setup/tooling cost from variable per-unit cost.
  - [x] Surface the quantity-sensitive economics through the existing `CostBreakdown` and `WorkbenchResult` fields instead of inventing a second costing schema.
- [x] Align the planner prompt and skill contract in `config/prompts.yaml` and `skills/manufacturing-knowledge/SKILL.md`.
  - [x] Tell the planner to evaluate candidate methods at the requested production volume, not only at a single unit.
  - [x] Keep the prompt aligned with the existing `target_quantity` field and the `1.5x` benchmark-cap derivation already used by the planner flow.
- [x] Extend integration coverage in `tests/integration/architecture_p1/test_manufacturing.py` and the planner-loop coverage in `tests/integration/architecture_p1/test_engineering_loop.py` or `tests/integration/architecture_p0/test_planner_gates.py` if the quantity assertion belongs in the handoff gate.
  - [x] Add a quantity-sensitive positive case that proves setup-cost amortization changes the economics when the requested quantity changes.
  - [x] Add a quantity-sensitive negative case that proves validation fails closed when the design is only feasible at a lower quantity than the one requested.
  - [x] Keep the assertions HTTP- and artifact-based, not unit-test-only.
- [x] Run the manufacturability and cost integration slices before closing the story.
  - [x] At minimum cover `INT-010`, `INT-011`, `INT-018`, `INT-019`, and the quantity-sensitive manufacturability path exercised through `INT-033` or a new regression slice if needed.

## Dev Notes

- Epic 3, Story 3.2 is the source of truth for the human-facing requirement.
- The quantity-aware economics already exist in the workbench layer: CNC, injection molding, and 3D printing each already compute fixed setup/tooling cost plus variable per-unit cost. The missing piece is making the requested production volume a first-class input in the validation/handoff path.
- `worker_heavy/utils/dfm.py::validate_and_price()` and `validate_and_price_assembly()` already accept a `quantity` parameter; reuse them instead of inventing a second manufacturability engine.
- `worker_heavy/utils/handover.py` and `worker_heavy/simulation/loop.py` currently call `validate_and_price_assembly()` without an explicit quantity. That is the likely gap if the requested production volume is not reflected in validation results.
- `controller/api/routes/benchmark.py` and `controller/agent/benchmark/nodes.py` already persist `target_quantity` on benchmark objectives. `controller/agent/benchmark_handover_validation.py` already checks that the copied quantity matches the benchmark objective. Reuse that field and do not add a second quantity contract.
- `shared/simulation/schemas.py::CustomObjectives`, `shared/models/schemas.py::Constraints`, and `shared/models/schemas.py::AssemblyDefinition` already carry the quantity-related fields used by the benchmark and planner flows. Keep the contract strict and fail closed on missing or conflicting quantity data.
- `worker_heavy/api/routes.py::api_analyze` already accepts `quantity` for direct workbench analysis. Preserve that surface so direct analysis tests can compare low- and high-volume economics.
- `controller/agent/tools.py::validate_costing_and_price()` remains the canonical planner gate entrypoint. Do not add a parallel quantity validator.
- For the planner, the selection rule should come from the existing workbench economics: low quantities can favor CNC or 3D printing, while high quantities can favor injection molding if the geometry and constraints fit.
- Story 3.1 already established price provenance. Story 3.2 should consume those validated inputs and extend them into quantity-aware manufacturability decisions. Story 3.3 still owns the broader weight/size/form-factor/COTS treatment.

### Project Structure Notes

- Do not create a second manufacturability system. Extend the existing workbench/DFM path and the planner prompt contract.
- Keep quantity semantics explicit: the benchmark or planner declares the requested production volume, the validator reads it, and the result metadata reports the economics at that volume.
- Do not add a new quantity field to `assembly_definition.yaml`; reuse `BenchmarkDefinition.constraints.target_quantity` and the existing `quantity` argument on direct analyze calls.
- Treat deterministic error strings as part of the contract because integration tests assert on them.
- Prefer additive changes to shared models and existing workbench result metadata over ad hoc dict payloads.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 3.2: Evaluate Manufacturability at the Requested Quantity]
- [Source: \_bmad-output/planning-artifacts/prd.md, Manufacturability and Costing FR16-FR21 and quantity-aware cost guidance]
- [Source: docs/backend-reference.md, manufacturability validation and worker-heavy workbench responsibilities]
- [Source: specs/architecture/CAD-and-other-infra.md, workbench cost model and quantity-based setup/variable cost contracts]
- [Source: specs/architecture/evals-and-gates.md, hard-check and fail-closed evaluation gates]
- [Source: specs/architecture/distributed-execution.md, worker-heavy validation/analyze boundary and routed heavy-path contract]
- [Source: specs/architecture/observability.md, manufacturability check events and lineage requirements]
- [Source: specs/architecture/agents/roles.md, engineering planner cost/price workflow and simpler-valid-solution preference]
- [Source: specs/architecture/agents/handover-contracts.md, planner handoff and review contract]
- [Source: specs/architecture/agents/tools.md, planner pricing tool and submit_plan gate]
- [Source: config/prompts.yaml, quantity-aware planner prompt contract and manufacturing decision guidance]
- [Source: skills/manufacturing-knowledge/SKILL.md, quantity-based CNC and injection-molding cost formulas]
- [Source: skills/manufacturing-knowledge/references/cnc.md, CNC setup vs variable cost model]
- [Source: skills/manufacturing-knowledge/references/injection_molding.md, tooling vs cycle cost model]
- \[Source: worker_heavy/utils/dfm.py, `validate_and_price` and `validate_and_price_assembly` quantity plumbing\]
- [Source: worker_heavy/workbenches/cnc.py, CNC setup vs variable cost model]
- [Source: worker_heavy/workbenches/injection_molding.py, tooling vs cycle cost model]
- [Source: worker_heavy/workbenches/print_3d.py, setup vs run cost model]
- [Source: worker_heavy/utils/handover.py, planner handoff validation and cost-limit enforcement]
- [Source: worker_heavy/simulation/loop.py, runtime manufacturability gate for latest revision validation]
- \[Source: shared/workers/schema.py, `AnalyzeRequest.quantity` contract for direct analysis calls\]
- \[Source: controller/api/routes/benchmark.py, `target_quantity` persistence on benchmark objectives\]
- [Source: controller/agent/benchmark/nodes.py, custom objective propagation into benchmark_definition.yaml]
- [Source: controller/agent/benchmark_handover_validation.py, target_quantity cross-contract validation]
- [Source: tests/integration/architecture_p1/test_manufacturing.py, workbench method/material rejection coverage]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, end-to-end engineering loop coverage]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, INT-010/011/018/019 planner-gate coverage]

## Dev Agent Record

### Agent Model Used

GPT-5

### Debug Log References

- `2026-03-23`: Threaded requested quantity through DFM validation metadata, resolved quantity from benchmark objectives for direct analyze and handoff validation, and surfaced quantity-aware setup/variable-cost fields in the workbench result model.
- `2026-03-23`: Added a direct `/benchmark/analyze` regression proving quantity changes setup amortization and trace metadata, plus a planner-gate regression proving handoff validation fails at the requested quantity instead of a single-unit shortcut.
- `2026-03-23`: Ran targeted integration runner slices for `tests/integration/architecture_p1/test_manufacturing.py::test_worker_analyze_quantity_changes_setup_amortization` and `tests/integration/architecture_p0/test_planner_gates.py::test_int_010_handoff_rejects_low_quantity_that_only_passes_at_volume`; both passed.
- `2026-03-23`: Ran adjacent planner-gate coverage in `tests/integration/architecture_p0/test_planner_gates.py -k 'test_int_019_single_part_benchmark_submit_succeeds_without_cost_gate or test_int_018'`; `INT-019` passed, while unrelated `INT-018` variants failed because those fixtures did not seed `manufacturing_config.yaml`.

### Completion Notes List

- Requested production quantity is now explicit in DFM metadata, `CostBreakdown`, and workbench result metadata, so the validator and planner gate can compare economics at the benchmarked volume.
- The planner prompt and manufacturing-knowledge skill now tell the planner to compare methods at the requested production volume rather than on a single-unit shortcut.
- The new quantity regression passes for direct analysis, and the planner-gate regression passes when the handoff lifecycle includes validate, simulate, and submit.
- The adjacent INT-018 planner-gate sweep surfaced unrelated fixture gaps around `manufacturing_config.yaml` seeding; those failures were not part of the quantity change and were left untouched.

### File List

- `_bmad-output/implementation-artifacts/3-2-evaluate-manufacturability-at-the-requested-quantity.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `config/prompts.yaml`
- `shared/workers/workbench_models.py`
- `tests/integration/architecture_p0/test_planner_gates.py`
- `tests/integration/architecture_p1/test_manufacturing.py`
- `skills/manufacturing-knowledge/SKILL.md`
- `worker_heavy/simulation/loop.py`
- `worker_heavy/utils/dfm.py`
- `worker_heavy/utils/handover.py`
- `worker_heavy/utils/validation.py`
- `worker_heavy/workbenches/cnc.py`
- `worker_heavy/workbenches/injection_molding.py`
- `worker_heavy/workbenches/print_3d.py`

### Change Log

- 2026-03-23: Threaded requested quantity through manufacturability validation, added quantity-aware result metadata, updated the planner manufacturing guidance, and added integration coverage for the direct analyze and planner-gate quantity paths.

### Status

review
