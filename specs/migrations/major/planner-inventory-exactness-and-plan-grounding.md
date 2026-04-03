# Planner Inventory Exactness and Plan Grounding Migration

<!-- Major migration. The eval refresh and integration refresh are the long pole. -->

## Purpose

This migration makes planner handoffs inventory-exact and plan-grounded.
Every planner-declared label and selected COTS `part_id` must appear in
`plan.md` as an exact identifier mention, with backticks preferred but not
required for validity. The planner-authored evidence and technical-drawing
scripts must preserve the same labels, repeated quantities, and COTS
identities as the approved inventory, and downstream coder/reviewer gates must
enforce the same contract.

Engineering planner handoffs also require a source-backed planning proof
layer: `plan.md` must carry an Assumption Register, Detailed Calculations, and
Critical Constraints / Operating Envelope sections so binding numeric claims
can be audited before handoff.

This migration also treats two validator gaps as bugs: deterministic weight
exactness and drafting geometry validity must be fixed, not parked as future
work.

The target contract is defined in:

- [Agent handovers](../../architecture/agents/handover-contracts.md)
- [Roles](../../architecture/agents/roles.md)
- [COTS geometry import](../../architecture/cots-geometry-import.md)

This migration applies to both benchmark and engineering flows.

## Problem Statement

The repository already states that planner inventories are binding, but the
enforcement is still fragmented.

1. Some handoff gates validate structure or geometry without proving that the
   authored inventory is preserved exactly.
2. `plan.md` coverage is specified as a contract, but the exact-mention rule
   still needs consistent enforcement across planner self-validation, reviewer
   gates, and coder intake.
3. Benchmark and engineering flows should behave the same way, but their eval
   rows, seed fixtures, and integration tests still encode older looser
   assumptions.
4. Because this contract spans planner, reviewer, and coder boundaries, the
   eval refresh is broad and the integration refresh is the long pole.
5. The current cost/weight contract is also buggy:
   - declared assembly cost is validated as a deterministic sum,
   - planner weight is only checked as a cap/ceiling today,
   - manufactured parts still need their per-part weight contributions carried
     through the deterministic validation path,
   - COTS parts are the only assembly rows that can optionally carry `weight_g`.
6. The drafting-artifact validation path is also buggy:
   - `solution_plan_evidence_script.py` and
     `benchmark_plan_evidence_script.py` are validated for inventory grounding,
     but physical validity must be a hard failure too,
   - `solution_plan_technical_drawing_script.py` and
     `benchmark_plan_technical_drawing_script.py` are validated for structure,
     inventory grounding, and `TechnicalDrawing` usage only, but that is not
     the full geometry contract,
   - self-intersection and overlap checks must be treated as bug fixes, not
     future notes.
7. Engineering planner handoffs still let binding numeric claims live only in
   prose. That makes slope thresholds, actuator load limits, and power budgets
   hard to audit before handoff.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `specs/architecture/agents/handover-contracts.md` | Defines the binding inventory contract and exact-mention rule. | The runtime and tests still need to enforce it consistently. |
| `specs/architecture/agents/roles.md` | States that benchmark and engineering planners/reviewers/coders must honor inventory exactness. | The role docs are ahead of the implementation and eval coverage. |
| `shared/assets/template_repos/engineer/plan.md`, `shared/workers/markdown_validator.py`, `controller/api/routes/episodes.py`, `controller/api/tasks.py`, `config/prompts.yaml` | The engineering plan surface carries the eight-section shape, and the validator, prompt text, and replay/normalization paths are the enforcement points for that contract. | The three proof sections stay useful only if the same exact headings and validation rules are enforced at prompt time, submit time, and replay time. |
| `controller/agent/node_entry_validation.py` | Validates node entry and some handoff constraints. | It needs fail-closed exactness gates for planner handoff packages. |
| `worker_heavy/utils/file_validation.py` | Validates schema and placeholder hygiene. | It must also participate in exact-mention and inventory-multiset checks where planner artifacts are validated. |
| `worker_heavy/utils/handover.py` | Collects and packages handoff artifacts. | It must reject mismatched inventory before a coder or reviewer sees it. |
| `skills/manufacturing-knowledge/scripts/validate_and_price.py`, `shared/models/schemas.py`, `worker_heavy/utils/dfm.py`, `controller/agent/benchmark/tools.py` | Define and enforce the current cost/weight contract. | They normalize cost but still leave weight as a planner-authored ceiling/estimate instead of a deterministic total. |
| `worker_heavy/utils/file_validation.py`, `controller/agent/node_entry_validation.py`, `worker_heavy/utils/validation.py` | Validate planner-authored evidence and technical-drawing scripts. | They still need to make physical self-intersection/overlap a first-class handoff failure instead of a follow-up note. |
| `shared/utils/agent/__init__.py` | Loads scripts and render evidence. | It needs to participate in the exactness contract for planner-authored evidence scripts. |
| `config/prompts.yaml`, `controller/agent/prompt_manager.py` | Planner prompt assembly does not yet teach exact-mention and inventory-exactness self-checks explicitly enough. | The planner needs the exactness contract in its guidance path so it can self-validate before handoff. |
| `evals/logic/specs.py`, `evals/logic/codex_workspace.py`, `scripts/validate_eval_seed.py` | Seeded evals still assume older artifact shapes in many rows. | All affected evals need the new contract so stale rows fail during preflight. |
| `tests/integration/**` | Many planner, reviewer, coder, and render-evidence tests still assert the old loose behavior. | Most integration tests that touch handoffs must be refreshed to exercise exactness. |
| `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py` | Planner-authored scripts exist, but their exactness checks are not yet the central fail-closed gate everywhere. | They are the canonical evidence of the contract and must be validated consistently. |

## Proposed Target State

1. Every planner handoff is inventory-exact. The evidence and technical-drawing
   scripts preserve the approved label multiset, repeated quantities, and COTS
   identities exactly.
2. Every planner-declared inventory label and selected COTS `part_id` appears
   in `plan.md` at least once as an exact identifier mention; backticks are
   preferred for the first mention, but the exact identifier match is the
   validation rule.
3. Planner self-validation, plan review, coder intake, and execution review all
   enforce the same exactness contract for benchmark and engineering flows.
4. Eval seeds, mock responses, and integration tests all encode the same
   inventory and plan-grounding rules so regressions fail before downstream
   implementation work starts.
5. Technical-drawing rendering and starter-template work remain separate from
   this migration; this migration owns the inventory/grounding contract, not
   the view-generation contract.
6. The cost/weight bug is fixed here as a separate exactness requirement so
   future contract work cannot reintroduce the asymmetry or confuse it with
   inventory-exactness work.
7. The drafting-geometry bug is fixed here as a separate requirement so
   future geometry work cannot relegate self-intersection or overlap checks to
   follow-up notes.
8. Engineering planner `plan.md` includes an Assumption Register, Detailed
   Calculations, and Critical Constraints / Operating Envelope section, so
   binding numeric claims are traceable from source-backed assumptions through
   derivations into reviewable operating limits.

## Required Work

### 1. Add shared exactness validation

- Implement or reuse a shared exact-mention validator for `plan.md`.
- Make planner-authored evidence and technical-drawing scripts self-validate
  the approved inventory before `submit_plan()`.
- Make planner handoff validation fail closed on missing labels, extra labels,
  quantity drift, or COTS identity drift.
- Ensure the same exactness rule is applied in both benchmark and engineering
  flows.

### 2. Tighten coder and reviewer gates

- Update plan-reviewer and execution-reviewer checks so they compare the
  current handoff against the approved inventory rather than a merely similar
  model.
- Make coder intake reject relabeled, missing, or extra inventory items.
- Keep the benchmark and engineering coder/reviewer contract aligned so the
  same defect is rejected in both flows.

### 3. Refresh eval seeds and mocks

- Update every seeded eval row that exercises planner, reviewer, coder, or
  render-evidence paths to use the new exactness contract.
- Refresh `evals/logic/specs.py`, `evals/logic/codex_workspace.py`, and
  role-scoped mock responses so they carry the inventory-exactness
  assumptions explicitly.
- Update `scripts/validate_eval_seed.py` and any seed fixtures that still
  permit the old loose handoff shape.
- Keep benchmark and engineering eval rows in parity where the artifact shape
  is the same.

### 4. Refresh integration coverage

- Update the planner, reviewer, and coder integration tests that assert
  handoff validity.
- Add explicit negative cases for missing exact mentions, label drift,
  quantity drift, and COTS `part_id` drift.
- Expect most integration tests that touch planner or coder handoffs to need
  edits; this migration is intentionally broad.

### 5. Align docs, prompts, and templates

- Keep the handoff, role, and prompt docs consistent with the exactness
  contract.
- Ensure `config/prompts.yaml`, `PromptManager`, and the plan templates tell
  planners to self-check exact mentions, inventory multiplicity, assumptions,
  calculations, and operating-envelope limits before handoff.
- Update the engineering planner plan template and markdown validator to
  require the new proof sections for binding numeric claims.

### 6. Fix the cost/weight exactness bug

- Add or reuse a deterministic weight validator alongside the deterministic
  cost validator.
- Compute weight from the actual assembly breakdown, including manufactured-part
  contributions and COTS weights, instead of treating it as a prose estimate.
- Keep the planner target cap as an envelope only after deterministic totals are
  validated.
- If a schema or script gap blocks exact weight validation, close it in this
  migration instead of deferring it.

### 7. Fix the drafting-artifact physical-validation bug

- Make planner-authored drafting geometry fail closed on self-intersection,
  overlap, and other invalid physical geometry.
- Keep inventory grounding and `TechnicalDrawing` structure checks, but do not
  let them stand in for physical validity.
- If the current file-level validator path misses a geometry case, add it now
  rather than documenting it as future work.

## Non-Goals

- No change to technical-drawing view generation, template scaffolds, or
  raster/vector rendering behavior.
- No full GD&T expansion.
- No change to the COTS catalog or catalog-search behavior.
- No broad rewrite of the mechanism narrative in `plan.md`; the migration only
  adds the exact-mention requirement and the planning proof sections.

## Sequencing

The safe order is:

1. Add the shared exactness validator and planner self-checks.
2. Tighten plan-reviewer, coder, and execution-reviewer gates.
3. Refresh eval seeds and mock responses.
4. Refresh the broad integration-test surface.
5. Tune prompts and docs after the contract is enforced in code.

## Acceptance Criteria

1. A planner handoff fails if `plan.md` is missing an exact mention for any
   declared inventory label or selected COTS `part_id`.
2. A planner handoff fails if the evidence or technical-drawing scripts omit,
   relabel, duplicate, or change the quantity of any declared inventory item.
3. Plan reviewers, coders, and execution reviewers reject the same inventory
   mismatch in both benchmark and engineering flows.
4. Seeded evals that still assume the old loose artifact shape fail during
   preflight rather than silently passing.
5. The updated integration suite covers the exact-mention, quantity, label,
   and COTS identity failure modes for both planner families.
6. A planner handoff fails if the written weight total does not match the
   deterministic total from the priced assembly breakdown.
7. Planner-authored drafting artifacts fail if their geometry self-intersects,
   overlaps, or otherwise violates the physical-validity contract, even when
   inventory grounding and `TechnicalDrawing` usage pass.
8. A planner handoff fails if a binding numeric claim in `plan.md` lacks a
   source-backed assumption, stable calculation ID, or operating-envelope
   limit.
9. An engineering planner handoff fails if `plan.md` is missing or malformed
   with respect to `Assumption Register`, `Detailed Calculations`, or
   `Critical Constraints / Operating Envelope`.

## Migration Checklist

### Contract and validation

- [x] Add the shared exact-mention validator for `plan.md`.
- [x] Add or reuse inventory-multiset validation for planner-authored
  evidence and technical-drawing scripts.
- [x] Make planner handoff validation fail closed on missing labels, quantity
  drift, relabeling, and COTS identity drift.
- [x] Apply the same exactness gate to benchmark and engineering handoffs.
- [x] Add the engineering planner proof sections to the template and heading
  validator: `Assumption Register`, `Detailed Calculations`, and `Critical Constraints / Operating Envelope`.
- [ ] Make binding numeric claims fail closed unless they carry a source-backed
  assumption, a stable calculation ID, and an operating-envelope limit.

### Plan review and coder intake

- [x] Update benchmark plan-reviewer gates to enforce exact mention and
  inventory preservation.
- [x] Update engineering plan-reviewer gates to enforce exact mention and
  inventory preservation.
- [x] Update benchmark coder intake and reviewer checks to reject inventory
  mismatches.
- [x] Update engineering coder intake and execution-reviewer checks to reject
  inventory mismatches.

### Eval refresh

- [ ] Update every seeded eval row that exercises planner, reviewer, coder,
  or render-evidence paths.
- [ ] Refresh `evals/logic/specs.py`, `evals/logic/codex_workspace.py`, and
  role-scoped mock responses for the new contract.
- [ ] Update `scripts/validate_eval_seed.py` and any seed fixtures that still
  assume the old loose handoff shape.
- [ ] Keep benchmark and engineering evals in parity where the artifact shape
  is the same.
- [ ] Update eval coverage for the `preview_drawing()` gate so drafting-mode
  planner, coder, and reviewer rows fail closed when the current revision does
  not record a required preview call.
- [ ] Update eval coverage for structural `TechnicalDrawing` validation so
  `solution_plan_technical_drawing_script.py` and
  `benchmark_plan_technical_drawing_script.py` fail when they do not truly
  import and invoke build123d `TechnicalDrawing`.

### Integration refresh

- [ ] Update the planner, reviewer, and coder integration tests that assume
  labels, quantities, or COTS identities may drift.
- [ ] Add negative-path coverage for missing exact mentions in `plan.md`.
- [ ] Add negative-path coverage for relabeling, quantity drift, and COTS
  identity drift in planner handoffs.
- [ ] Refresh most integration tests that touch planner or coder handoffs.
- [ ] Add integration-test coverage for the `preview_drawing()` gate in the
  drafting-mode planner, coder, and reviewer flows.
- [ ] Add integration-test coverage for structural `TechnicalDrawing`
  validation on `solution_plan_technical_drawing_script.py` and
  `benchmark_plan_technical_drawing_script.py`.

### Documentation and prompts

- [ ] Align prompt-side guidance with the exactness contract.
- [ ] Keep the handoff and role docs in sync with the implemented validator.

### Cost/weight exactness bug

- [ ] Add deterministic exact-weight validation next to the deterministic cost
  check so planner handoffs fail if the written weight total does not match
  the computed total.
- [ ] Keep planner caps as ceilings only after exact weight is established.
- [ ] Close any schema or script gap needed to compute weight from the actual
  part breakdown instead of deferring it to a future migration.

### Drafting physical-validation bug

- [ ] Make planner drafting geometry fail closed on self-intersection, overlap,
  and other invalid geometry cases.
- [ ] Keep inventory grounding and `TechnicalDrawing` structure checks, but do
  not treat them as the whole geometry contract.
- [ ] If the validator path is missing a geometry case, add it here instead of
  marking it as follow-up.

### Drafting contract follow-up

- [x] Add the fail-closed `preview_drawing()` usage gate for drafting-mode planner, coder, and reviewer nodes, and wire it through the drafting, tool, and prompt docs.
- [x] Add structural validation for `solution_plan_technical_drawing_script.py` and `benchmark_plan_technical_drawing_script.py` so the scripts must truly import and invoke build123d `TechnicalDrawing`, using AST/symbol checks rather than substring matching.

## File-Level Change Set

The implementation should touch the smallest set of files that enforce the
new contract:

- `controller/agent/node_entry_validation.py`
- `skills/manufacturing-knowledge/scripts/validate_and_price.py`
- `shared/assets/template_repos/engineer/plan.md`
- `shared/workers/markdown_validator.py`
- `controller/api/routes/episodes.py`
- `worker_heavy/utils/file_validation.py`
- `worker_heavy/utils/validation.py`
- `worker_heavy/utils/handover.py`
- `shared/utils/agent/__init__.py`
- `scripts/validate_eval_seed.py`
- `evals/logic/specs.py`
- `evals/logic/codex_workspace.py`
- `benchmark_plan_evidence_script.py`
- `benchmark_plan_technical_drawing_script.py`
- `solution_plan_evidence_script.py`
- `solution_plan_technical_drawing_script.py`
- `tests/integration/**`
- `config/prompts.yaml`
- `controller/agent/prompt_manager.py`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/roles.md`
- `specs/architecture/cots-geometry-import.md`
- `specs/migrations/minor/engineering-planner-technical-drawings-migration.md`
