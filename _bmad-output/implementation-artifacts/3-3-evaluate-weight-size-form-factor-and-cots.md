# Story 3.3: Evaluate Weight, Size, Form Factor, and COTS

Status: ready-for-dev

## Story

As a human operator, I want the system to evaluate weight, size, and form-factor constraints together, and include COTS parts in the cost and manufacturability result, so that the answer reflects what can actually be built.

## Acceptance Criteria

1. Given a design that includes catalog-backed COTS parts, when the solution is priced and weighed, then the COTS parts are counted in the totals using concrete catalog `part_id`, `manufacturer`, `source`, `unit_cost`, and `weight_g` provenance rather than invented values.
1. Given a design that violates size, weight, or form-factor limits, when validation runs, then the system rejects it with an explicit reason that names the violated constraint and the offending part or subassembly.
1. Given a candidate COTS part without a valid catalog or manufacturing reference, when the design is reviewed or submitted, then the system fails closed instead of assuming a price, weight, or geometry fit.
1. Given a mixed assembly of manufactured parts and COTS parts, when planner validation, node-entry revalidation, and runtime handoff checks run, then the same deterministic assembly source of truth is used and the persisted artifacts round-trip without drift.
1. Given benchmark-owned fixtures or benchmark-owned electronics that carry `cots_id`, when engineer totals are computed, then those read-only fixtures are excluded from engineer-owned cost and weight totals.
1. Given the same COTS part is reused multiple times, when the assembly is priced, then the quantity remains explicit and the total remains stable across planner handoff, validation, and review.

## Tasks / Subtasks

- [ ] Extend the DFM and assembly totals path in `worker_heavy/utils/validation.py`, `worker_heavy/utils/dfm.py`, and `worker_heavy/utils/handover.py` so catalog-backed COTS parts contribute both cost and weight through the same deterministic path as manufactured parts.
  - [ ] Use catalog-backed `weight_g` from `shared/cots/runtime.py` or `shared/cots/indexer.py`, or from persisted COTS provenance, rather than hardcoded heuristics.
  - [ ] Keep build-zone and workbench-specific size/form-factor violations fail-closed with deterministic error text.
- [ ] Preserve COTS provenance in the shared schema and planner scaffold.
  - [ ] If the current `CotsPartEstimate` or template fields cannot carry the needed weight or snapshot metadata, extend the existing model and `shared/assets/template_repos/engineer/assembly_definition.yaml` in place instead of inventing a parallel COTS record format.
  - [ ] Keep `part_id`, `manufacturer`, `source`, `unit_cost_usd`, `weight_g`, `quantity`, and the catalog snapshot metadata traceable.
- [ ] Keep the planner and skill contract aligned for COTS and size/form-factor reasoning.
  - [ ] Verify `config/prompts.yaml` and `skills/manufacturing-knowledge/SKILL.md` still tell the planner to use exact catalog IDs, the requested quantity, and the actual weight/size constraints.
  - [ ] Only change prompt text if the current wording leaves a gap after the runtime contract is updated.
- [ ] Tighten fail-closed validation for missing catalog references and unresolved COTS entries.
  - [ ] Update `worker_heavy/utils/file_validation.py` and `controller/agent/node_entry_validation.py` if the new COTS fields require schema acceptance or additional cross-contract rejection before review submission.
- [ ] Extend integration coverage in `tests/integration/architecture_p0/test_planner_gates.py`, `tests/integration/architecture_p1/test_manufacturing.py`, and `tests/integration/architecture_p1/test_engineering_loop.py` or `tests/integration/architecture_p1/test_handover.py`.
  - [ ] Add a positive case that proves COTS weight is included in the totals and catalog provenance is persisted.
  - [ ] Add a negative case that proves an unresolved COTS reference or oversized form factor fails closed with a specific reason.
  - [ ] Keep assertions HTTP- and artifact-based, not unit-test-only.
- [ ] Run the relevant integration slices before closing the story.
  - [ ] At minimum cover `INT-010`, `INT-012`, `INT-013`, `INT-014`, `INT-018`, `INT-019`, `INT-033`, and `INT-064`; add a regression slice if the COTS-weight persistence path needs one.

## Dev Notes

- Epic 3, Story 3.3 is the source of truth for the human-facing requirement.
- Story 3.1 already owns price provenance. Story 3.2 already owns quantity-aware manufacturability. Do not duplicate either contract here.
- `shared.models.schemas.COTSMetadata` already carries `part_id`, `name`, `category`, `unit_cost`, `weight_g`, `bbox`, `volume`, and `params`. `CotsPartEstimate` is the planner artifact, and `AssemblyDefinition` is the planner handoff. If COTS weight or snapshot metadata is missing from the planner artifact, extend the existing schema or template rather than introducing a second COTS record shape.
- `worker_heavy/utils/validation.py::calculate_assembly_totals()` currently adds cost for `cots_parts` but does not clearly add COTS weight. That is the most likely gap if totals are undercounted.
- `worker_heavy/utils/handover.py::submit_for_review()` is the fail-closed submission gate for cost and weight. Keep any new COTS-weight logic deterministic there.
- `worker_heavy/utils/dfm.py::validate_and_price_assembly()` remains the canonical manufacturability path. Form-factor and build-zone violations should continue to come from the existing workbench and validation path, not a second validator.
- `controller/agent/tools.py::validate_costing_and_price()` and `submit_plan()` remain the planner gate. Do not bypass them.
- `shared/cots/runtime.py`, `shared/cots/indexer.py`, and the COTS catalog contract in `specs/architecture/agents/roles.md` define the reproducibility metadata (`catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`, query snapshot, selection snapshot). Preserve those fields if you surface or extend COTS provenance in planner artifacts.
- Benchmark-owned fixtures and benchmark-owned electronics remain read-only context even when they carry `cots_id`; they must not be counted as engineer-owned COTS totals.
- The planner prompt already covers exact COTS IDs and quantity; update `config/prompts.yaml` or the manufacturing skill only if the current wording still leaves ambiguity after the runtime contract is tightened.

### Project Structure Notes

- Keep COTS provenance in shared schema models and catalog helpers. Do not route weight handling through ad hoc dicts or prompt prose.
- Keep form-factor checks inside the existing workbench and validation pipeline. Do not create a separate size subsystem.
- Preserve strict schema and fail-closed behavior on missing catalog IDs, missing weights, build-zone violations, and unresolved catalog snapshots.
- Treat benchmark-owned read-only fixtures as excluded from engineer totals even when they have catalog identity.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 3.3: Evaluate Weight, Size, Form Factor, and COTS]
- [Source: \_bmad-output/planning-artifacts/prd.md, Technical Constraints and Manufacturability and Costing FR19-FR21]
- [Source: specs/desired_architecture.md, architecture index for agents, evaluation gates, and CAD/infrastructure contracts]
- [Source: specs/architecture/primary-system-objectives.md, product-level engineering and dataset goals]
- [Source: specs/architecture/agents/overview.md, engineering workflow split and review ordering]
- [Source: specs/architecture/agents/roles.md, engineering planner intake, COTS catalog database, and planner/reviewer responsibilities]
- [Source: specs/architecture/agents/handover-contracts.md, planner handoff contracts, read-only benchmark fixtures, and COTS provenance rules]
- \[Source: specs/architecture/agents/tools.md, `validate_costing_and_price`, `submit_plan`, and planner COTS rules\]
- [Source: specs/architecture/CAD-and-other-infra.md, part metadata, workbench contracts, and COTS read-only catalog assumptions]
- [Source: specs/architecture/evals-and-gates.md, fail-closed planner and review gates plus quantitative manufacturability expectations]
- [Source: specs/architecture/observability.md, COTS search and cost/weight observability requirements]
- [Source: docs/backend-reference.md, backend runtime split, engineering workflow, and COTS/DFM contracts]
- \[Source: shared/models/schemas.py, `COTSMetadata`, `CotsPartEstimate`, and `AssemblyDefinition`\]
- [Source: shared/cots/runtime.py, COTS catalog search and reproducibility metadata]
- [Source: shared/cots/indexer.py, catalog weight and part metadata extraction]
- \[Source: worker_heavy/utils/validation.py, `calculate_assembly_totals` and runtime weight aggregation\]
- \[Source: worker_heavy/utils/dfm.py, `validate_and_price` and `validate_and_price_assembly`\]
- [Source: worker_heavy/utils/handover.py, submission-stage cost/weight/build-zone gate]
- [Source: worker_heavy/utils/file_validation.py, planner handoff validation and cross-contract checks]
- [Source: controller/agent/node_entry_validation.py, planner handoff schema validation and cross-contract gating]
- \[Source: controller/agent/tools.py, `validate_costing_and_price` and `submit_plan`\]
- [Source: config/prompts.yaml, planner prompt contract and COTS selection rules]
- [Source: skills/manufacturing-knowledge/SKILL.md, cost-model guidance for CNC and injection molding]
- [Source: skills/manufacturing-knowledge/references/cnc.md, CNC stock-size and removed-volume cost logic]
- [Source: skills/manufacturing-knowledge/references/injection_molding.md, tooling, cycle, and wall-thickness logic]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, planner gate and COTS contract coverage]
- [Source: tests/integration/architecture_p1/test_manufacturing.py, workbench/material/COTS manufacturing coverage]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, end-to-end engineer loop coverage]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff coverage]
- [Source: specs/integration-tests.md, INT-010, INT-012, INT-013, INT-014, INT-018, INT-019, INT-033, INT-064]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
