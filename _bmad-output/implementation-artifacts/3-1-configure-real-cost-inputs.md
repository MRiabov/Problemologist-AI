# Story 3.1: Configure Real Cost Inputs

Status: ready-for-dev

## Story

As a human operator, I want to configure the prices used for costing from real manufacturing and catalog data so that the system evaluates designs with defensible price assumptions instead of invented numbers.

## Acceptance Criteria

1. Given a workspace `manufacturing_config.yaml` or catalog source with valid pricing data, when the planner validates costing inputs, then the selected price assumptions are loaded from that source, applied to the handoff, and persisted in the planner artifact bundle for later validation.
1. Given a missing, malformed, stale, or non-catalog price source, when costing inputs are validated, then the planner handoff is rejected with an explicit reason that names the missing or invalid source and no plan is submitted.
1. Given catalog-backed COTS items or material inputs, when planner artifacts are written, then the chosen `part_id`/manufacturer/source/unit-cost data remain traceable in `assembly_definition.yaml` and the episode record, rather than being replaced with inferred or invented values.

## Tasks / Subtasks

- [ ] Tighten the pricing-source path in `worker_heavy/workbenches/config.py`, `worker_heavy/utils/dfm.py`, and `controller/agent/tools.py` so planner validation and handoff submission both use the same merged manufacturing-config source of truth.
  - [ ] Fail closed when the workspace override is missing, unreadable, or cannot be merged into the repository config.
  - [ ] Keep the returned error text deterministic and specific enough for integration assertions.
- [ ] Keep the planner prompt contract aligned in `config/prompts.yaml`.
  - [ ] Tell the planner to read `manufacturing_config.yaml` or `/config/manufacturing_config.yaml`, use `invoke_cots_search_subagent(...)` for each planned COTS part, and run `validate_costing_and_price()` before `submit_plan()`.
  - [ ] Explicitly forbid invented costs or uncatalogued COTS IDs in planner-authored artifacts.
- [ ] Preserve and validate price provenance in the existing assembly schema and validation path.
  - [ ] Reuse `ManufacturedPartEstimate`, `CotsPartEstimate`, and the current `validate_assembly_definition_yaml` / `validate_planner_handoff_cross_contract` flow rather than inventing a second costing record format.
  - [ ] Ensure catalog-backed entries keep their source metadata and that later validation can replay the same pricing basis.
- [ ] Extend integration coverage in `tests/integration/architecture_p0/test_planner_gates.py` and `tests/integration/architecture_p1/test_manufacturing.py`.
  - [ ] Add a positive case that proves a real workspace cost override or catalog-backed price source is accepted and produces the expected totals.
  - [ ] Add a negative case that proves missing or malformed price data fails closed with an explicit reason.
  - [ ] Keep the assertions HTTP- and artifact-based, not unit-test-only.
- [ ] Run the cost-related integration slices before marking the story complete.
  - [ ] At minimum cover `INT-010`, `INT-012`, `INT-013`, `INT-014`, `INT-018`, and `INT-019`; include any follow-on regression needed to prove pricing provenance is stable.

## Dev Notes

- Epic 3, Story 3.1 is the source of truth for the human-facing requirement.
- The authoritative pricing path is `controller/agent/tools.py::validate_costing_and_price()` -> `worker_heavy/utils/dfm.py` -> `worker_heavy/workbenches/config.py`; do not add a second pricing service or infer prices from plan prose.
- `manufacturing_config.yaml` is a validated workspace override layered on top of the repository default. Missing or invalid overrides must fail closed rather than falling back to invented values.
- Catalog prices must come from the current episode's COTS lookup path and stay traceable through `part_id`, `manufacturer`, `source`, unit cost, and catalog snapshot metadata. If you persist provenance beyond `assembly_definition.yaml`, keep `catalog_version`, `bd_warehouse_commit`, `generated_at`, and `catalog_snapshot_id` with the same episode so later validation can replay the same price basis.
- Keep benchmark-owned fixtures read-only and out of engineer pricing totals; only engineer-owned manufactured parts and selected COTS components belong in the costing path.
- `submit_plan()` remains the final gate. If the cost source is missing or invalid, the planner must be rejected before handoff.
- Story 3.2 will consume the validated cost assumptions for quantity-aware manufacturability; do not bake quantity logic into this story.
- Story 3.3 will consume the same provenance to include COTS parts in full cost/weight evaluation; keep the contract reusable rather than specialized here.

### Project Structure Notes

- Keep cost-source validation in the existing config/DFM/planner-tool pipeline. Do not introduce a separate pricing database or a parallel config loader.
- Treat deterministic error strings as part of the contract because integration tests assert on explicit rejection reasons.
- Keep the planner prompt in sync with the actual runtime gates so the model does not rely on undocumented price sources.
- If a new provenance field is required, add it to the existing schema model rather than using ad hoc dict payloads.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 3.1: Configure Real Cost Inputs]
- [Source: \_bmad-output/planning-artifacts/prd.md, Functional Requirements FR16-FR21 and manufacturing/cost sections]
- [Source: specs/architecture/primary-system-objectives.md, product-level output goals for benchmark and engineering datasets]
- [Source: specs/architecture/agents/overview.md, engineering planner and reviewer workflow split]
- [Source: specs/architecture/agents/roles.md, engineering planner cost/price workflow and COTS usage]
- [Source: specs/architecture/agents/handover-contracts.md, planner handoff cost contract and COTS provenance rules]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, planner write scope and read-only benchmark context]
- [Source: specs/architecture/CAD-and-other-infra.md, manufacturing config, pricing, and COTS metadata contracts]
- [Source: specs/architecture/evals-and-gates.md, fail-closed planner submission and seeded preflight contract]
- [Source: specs/architecture/observability.md, catalog provenance and lineage/event requirements]
- [Source: docs/backend-reference.md, validate_and_price, COTS search, and manufacturing-config workflow]
- [Source: config/prompts.yaml, planner prompt contract and pricing validity requirements]
- [Source: worker_heavy/workbenches/config.py, manufacturing config loading and merge behavior]
- [Source: worker_heavy/utils/dfm.py, manufacturability and pricing analysis entrypoint]
- [Source: controller/agent/tools.py, planner pricing tool wrapper and submit_plan gate]
- [Source: worker_heavy/utils/file_validation.py, planner handoff validation and cross-contract checks]
- \[Source: shared/models/schemas.py, `ManufacturedPartEstimate`, `CotsPartEstimate`, and `AssemblyDefinition` schema contracts\]
- [Source: shared/workers/workbench_models.py, manufacturing config and cost-model definitions]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, planner pricing script integration and cost cap gates]
- [Source: tests/integration/architecture_p1/test_manufacturing.py, workbench material/material-rejection coverage]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
