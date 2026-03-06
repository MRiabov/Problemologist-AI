---
work_package_id: WP01
title: Entry Guard Foundation & Contracts
lane: "for_review"
dependencies: []
base_branch: main
base_commit: c500573985514e3935b8662f0792ab5d569ae5a7
created_at: '2026-03-06T08:45:30.024393+00:00'
subtasks:
- T001
- T002
- T003
- T004
- T005
phase: Phase 1 - Foundations
assignee: ''
agent: codex
shell_pid: '8707'
review_status: ''
reviewed_by: ''
review_feedback: ''
history:
- timestamp: '2026-03-06T08:36:05Z'
  lane: planned
  agent: system
  shell_pid: ''
  action: Prompt generated via /spec-kitty.tasks
requirement_refs:
- FR-001
- FR-002
- FR-005
- FR-013
- NFR-002
- C-003
---

# Work Package Prompt: WP01 - Entry Guard Foundation & Contracts

## Objective

Create the shared node-entry guard primitives that both orchestration graphs will consume. This WP must deliver typed contracts and deterministic routing policy building blocks before any graph wiring begins.

## Implementation Command

```bash
spec-kitty implement WP01
```

## Context & Constraints

- Feature docs:
  - `kitty-specs/012-automatic-node-entry-validation/spec.md`
  - `kitty-specs/012-automatic-node-entry-validation/plan.md`
  - `kitty-specs/012-automatic-node-entry-validation/research.md`
  - `kitty-specs/012-automatic-node-entry-validation/data-model.md`
  - `kitty-specs/012-automatic-node-entry-validation/contracts/node-entry-validation.yaml`
- Architecture constraints:
  - Fail-closed behavior is mandatory.
  - Integration mode must use fail-fast on entry rejection.
  - Deterministic previous-node mapping is required (no heuristics).
  - Subagent scope: first-class graph nodes only.

## Subtasks & Detailed Guidance

### T001 - Create shared validation module and typed models
- Purpose: Centralize all entry-validation result/error typing.
- Steps:
  1. Add a dedicated module under `controller/agent/` (for example `node_entry_validation.py`).
  2. Define typed models for validation result and validation errors.
  3. Ensure fields align with `data-model.md` (`ok`, `disposition`, `errors`, `reroute_target`, `reason_code`).
- Files:
  - `controller/agent/node_entry_validation.py` (new)
  - optional type exports if needed in `controller/agent/__init__.py`
- Parallel: No.
- Notes: Prefer existing enums over raw strings where available.

### T002 - Add disposition/source enums and reason constants
- Purpose: Provide stable machine-readable keys used by tests and logs.
- Steps:
  1. Add/extend enums for entry disposition and validation source.
  2. Add canonical reason code constants for common failures (`missing_artifact`, `state_invalid`, `handover_invalid`, etc.).
  3. Keep values backward-safe for trace consumers.
- Files:
  - `shared/enums.py` (if enum extension is needed)
  - or `controller/agent/node_entry_validation.py` constants
- Parallel: Yes, can proceed with T001 if imports are coordinated.
- Notes: Avoid introducing string drift across modules.

### T003 - Implement deterministic previous-node mapping tables
- Purpose: Make loopback routing explicit and testable.
- Steps:
  1. Define map(s) for engineer graph and benchmark graph previous-node targets.
  2. Include mapping for all first-class nodes in scope.
  3. Handle start-node/single-node edge cases explicitly (no previous node => fail-closed behavior defined).
- Files:
  - `controller/agent/node_entry_validation.py`
- Parallel: Yes.
- Notes: This map is a contract artifact. Keep it simple and static.

### T004 - Add integration policy resolver
- Purpose: One source of truth for runtime mode disposition policy.
- Steps:
  1. Add helper to detect integration mode via existing settings (`is_integration_test`).
  2. Expose helper for graph code to choose reroute vs fail-fast.
  3. Keep behavior deterministic and side-effect free.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/config.py` (only if additional setting access is needed)
- Parallel: Yes.
- Notes: Do not duplicate mode checks in multiple graphs.

### T005 - Create contract evaluation interface
- Purpose: Standardize how node-specific preconditions are expressed and evaluated.
- Steps:
  1. Add `NodeEntryContract` representation with required state fields, artifact list, and optional custom check hook.
  2. Implement evaluator that consumes contract + state + worker accessor context and returns typed result.
  3. Keep evaluator generic; graph-specific node contracts come in later WPs.
- Files:
  - `controller/agent/node_entry_validation.py`
- Parallel: No (depends on T001-T004 structures).
- Notes: Return structured errors; do not raise for routine contract failures.

## Test Strategy

Integration tests are implemented in later WPs. For this WP, verify locally that:
- Models/enums import cleanly in controller modules.
- Contract evaluation returns deterministic outputs for sample inputs.

## Risks & Mitigations

- Risk: Over-coupling this module to graph internals.
- Mitigation: Keep it pure/typed and pass graph-specific data in contracts.

- Risk: Inconsistent reason-code naming across future WPs.
- Mitigation: Define and document constants in this WP; treat as shared contract.

## Review Guidance

Reviewer should verify:
- All first-class typed primitives exist.
- No graph-specific business logic is hardcoded in foundation evaluator.
- Mapping/policy primitives are deterministic and easy to test.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
- 2026-03-06T08:45:30Z – codex – shell_pid=8707 – lane=doing – Assigned agent via workflow command
- 2026-03-06T08:56:04Z – codex – shell_pid=8707 – lane=for_review – Ready for review: added typed node-entry validation foundation, deterministic maps, and integration-policy resolver
