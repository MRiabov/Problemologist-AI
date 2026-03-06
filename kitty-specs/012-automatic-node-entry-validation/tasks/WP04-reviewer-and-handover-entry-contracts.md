---
work_package_id: WP04
title: Reviewer & Handover Entry Contracts
lane: planned
dependencies: [WP02, WP03]
subtasks:
- T018
- T019
- T020
- T021
- T022
phase: Phase 3 - Contract Harmonization
assignee: ''
agent: ''
shell_pid: ''
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
- FR-008
- FR-010
- FR-013
- C-004
- C-005
---

# Work Package Prompt: WP04 - Reviewer & Handover Entry Contracts

## Objective

Unify reviewer-entry validation behavior across engineer and benchmark flows by enforcing latest-revision handover invariants before reviewer node execution.

## Implementation Command

```bash
spec-kitty implement WP04 --base WP03
```

## Context & Constraints

- Primary utility: `controller/agent/review_handover.py`.
- Existing reviewer gates are fail-closed and must stay fail-closed.
- Entry-layer integration must not weaken manifest/hash/simulation/validation checks.
- Subagent exception boundary must remain explicit and documented.

## Subtasks & Detailed Guidance

### T018 - Reuse handover validator as contract hook
- Purpose: Avoid duplicate reviewer precheck logic.
- Steps:
  1. Wire `validate_reviewer_handover` into node-entry contract custom checks.
  2. Reuse for execution reviewer and benchmark reviewer guard paths.
  3. Normalize returned error into structured entry-validation error records.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/review_handover.py` (only if tiny adapter needed)
- Parallel: No.
- Notes: Keep current handover function signature stable if possible.

### T019 - Add explicit reviewer-entry contracts
- Purpose: Ensure reviewer nodes cannot execute without valid latest-revision handover.
- Steps:
  1. Define reviewer contracts in shared contract registry.
  2. Bind contracts to engineer execution reviewer and benchmark reviewer transitions.
  3. Ensure contract includes manifest presence + freshness checks via hook.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Preserve current reviewer decision schema gate behavior downstream.

### T020 - Normalize reviewer entry failure classification
- Purpose: Distinguish entry contract failures from in-node review failures.
- Steps:
  1. Introduce dedicated reason code(s) for reviewer entry block.
  2. Ensure traces/logs/journal reflect entry-phase rejection.
  3. Avoid ambiguous messages like generic internal error when precheck fails.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Enables clean integration assertions.

### T021 - Enforce subagent scope boundary
- Purpose: Keep feature scope aligned with spec (`FR-013`).
- Steps:
  1. Explicitly apply entry checks only to first-class graph nodes.
  2. Do not implicitly treat tool-invoked helper subagents as node transitions.
  3. Add TODO marker/comment for future dedicated subagent guard work if needed.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: This is a boundary contract, not a runtime workaround.

### T022 - Add clarifying guardrail comments/docstrings
- Purpose: Reduce future regressions from misunderstood boundaries.
- Steps:
  1. Add concise comments near contract registration and routing logic.
  2. Document integration fail-fast intent and why it differs from runtime loopback.
  3. Document reviewer handover hook reuse and no-fallback policy.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Keep comments brief and architecture-focused.

## Test Strategy

- Use integration scenarios where reviewer handover artifacts are missing/stale.
- Assert reviewer node is blocked before execution in both graphs.

## Risks & Mitigations

- Risk: Accidentally changing reviewer semantics beyond entry gating.
- Mitigation: Restrict changes to pre-entry path and keep in-node reviewer logic intact.

## Review Guidance

Reviewer should verify:
- Reviewer preconditions are checked at entry level.
- Failure messages are explicit and machine-assertable.
- Subagent scope statement is reflected in code comments/contracts.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
