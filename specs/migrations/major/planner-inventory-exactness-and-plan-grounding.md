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

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `specs/architecture/agents/handover-contracts.md` | Defines the binding inventory contract and exact-mention rule. | The runtime and tests still need to enforce it consistently. |
| `specs/architecture/agents/roles.md` | States that benchmark and engineering planners/reviewers/coders must honor inventory exactness. | The role docs are ahead of the implementation and eval coverage. |
| `controller/agent/node_entry_validation.py` | Validates node entry and some handoff constraints. | It needs fail-closed exactness gates for planner handoff packages. |
| `worker_heavy/utils/file_validation.py` | Validates schema and placeholder hygiene. | It must also participate in exact-mention and inventory-multiset checks where planner artifacts are validated. |
| `worker_heavy/utils/handover.py` | Collects and packages handoff artifacts. | It must reject mismatched inventory before a coder or reviewer sees it. |
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

### 5. Align docs and prompts

- Keep the handoff, role, and prompt docs consistent with the exactness
  contract.
- Ensure `config/prompts.yaml` and `PromptManager` tell planners to
  self-check exact mentions and inventory multiplicity before handoff.

## Non-Goals

- No change to technical-drawing view generation, template scaffolds, or
  raster/vector rendering behavior.
- No full GD&T expansion.
- No change to the COTS catalog or catalog-search behavior.
- No rewrite of the planner narrative structure in `plan.md` beyond the
  exact-mention requirement.

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

## Migration Checklist

### Contract and validation

- [x] Add the shared exact-mention validator for `plan.md`.
- [x] Add or reuse inventory-multiset validation for planner-authored
  evidence and technical-drawing scripts.
- [x] Make planner handoff validation fail closed on missing labels, quantity
  drift, relabeling, and COTS identity drift.
- [x] Apply the same exactness gate to benchmark and engineering handoffs.

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

### Integration refresh

- [ ] Update the planner, reviewer, and coder integration tests that assume
  labels, quantities, or COTS identities may drift.
- [ ] Add negative-path coverage for missing exact mentions in `plan.md`.
- [ ] Add negative-path coverage for relabeling, quantity drift, and COTS
  identity drift in planner handoffs.
- [ ] Refresh most integration tests that touch planner or coder handoffs.

### Documentation and prompts

- [ ] Align prompt-side guidance with the exactness contract.
- [ ] Keep the handoff and role docs in sync with the implemented validator.

## File-Level Change Set

The implementation should touch the smallest set of files that enforce the
new contract:

- `controller/agent/node_entry_validation.py`
- `worker_heavy/utils/file_validation.py`
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
