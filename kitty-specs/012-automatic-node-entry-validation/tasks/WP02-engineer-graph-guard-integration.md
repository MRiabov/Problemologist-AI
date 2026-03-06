---
work_package_id: WP02
title: Engineer Graph Guard Integration
lane: "doing"
dependencies: [WP01]
base_branch: main
base_commit: 22af09078a853762f9384a23494fafc08ae1d6fd
created_at: '2026-03-06T09:35:03.039089+00:00'
subtasks:
- T006
- T007
- T008
- T009
- T010
- T011
phase: Phase 2 - Engineer Runtime Integration
assignee: ''
agent: ''
shell_pid: "8707"
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
- FR-003
- FR-004
- FR-005
- FR-006
- FR-010
- FR-011
- NFR-001
- NFR-002
- C-003
---

# Work Package Prompt: WP02 - Engineer Graph Guard Integration

## Objective

Wire pre-entry validation into the engineer graph and single-node planner graphs so every first-class transition is gated before target node execution.

## Implementation Command

```bash
spec-kitty implement WP02 --base WP01
```

## Context & Constraints

- Core files:
  - `controller/agent/graph.py`
  - `controller/agent/state.py`
  - `controller/api/tasks.py`
- Must preserve existing node semantics while adding pre-entry safety.
- Do not remove post-node validators; this is additive defense.
- Integration mode must fail-fast on entry rejection.

## Subtasks & Detailed Guidance

### T006 - Add guard integration to engineer graph transitions
- Purpose: Ensure transitions invoke guard before target node execution.
- Steps:
  1. Introduce guard-aware routing functions in `controller/agent/graph.py`.
  2. Gate transitions into planner/electronics planner/reviewers/coders/electronics engineer/skill/journalling/steer according to node scope.
  3. Ensure target node is not called when guard returns rejection.
- Files:
  - `controller/agent/graph.py`
- Parallel: No.
- Notes: Keep routing readable; avoid branching explosion.

### T007 - Define engineer node contracts
- Purpose: Encode required state/artifact prerequisites per engineer node.
- Steps:
  1. Add contract definitions for all engineer first-class nodes.
  2. Include planner handoff prerequisites before coder entry.
  3. Add minimal contracts for nodes that have lighter preconditions (for example skills/journalling).
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/graph.py` (if contract registration happens there)
- Parallel: Yes.
- Notes: Contracts should be explicit and fail-closed.

### T008 - Implement non-integration reroute behavior
- Purpose: Preserve recoverability outside integration mode.
- Steps:
  1. On rejection, map to deterministic previous node.
  2. Update state feedback/journal fields with rejection reason.
  3. Ensure loopback target is stable for repeated same input.
- Files:
  - `controller/agent/graph.py`
- Parallel: No.
- Notes: No heuristics from transient traces.

### T009 - Implement integration fail-fast behavior
- Purpose: Eliminate integration loops on entry rejections.
- Steps:
  1. On rejection in integration mode, set terminal failure state directly.
  2. Populate failure reason and metadata for episode persistence/broadcast paths.
  3. Ensure flow exits cleanly without entering target node.
- Files:
  - `controller/agent/graph.py`
  - `controller/api/tasks.py` (only if required for completion status sync)
- Parallel: No.
- Notes: One-turn transition to failed outcome is required.

### T010 - Guard single-node planner graphs
- Purpose: Keep direct planner runs consistent with fail-closed semantics.
- Steps:
  1. Apply entry validation to `engineer_planner_graph` and `electronics_planner_graph`.
  2. Define behavior when no previous node exists (fail-closed).
  3. Ensure planner direct API runs surface clear rejection reason.
- Files:
  - `controller/agent/graph.py`
- Parallel: Yes.
- Notes: No loopback in single-node mode.

### T011 - Surface entry failure context to state/persistence
- Purpose: Make failures visible to status/traces consumers.
- Steps:
  1. Populate `feedback`, `journal`, and any relevant state fields with structured reason strings.
  2. Ensure data is compatible with existing episode update flow.
  3. Keep error messages stable for integration assertions.
- Files:
  - `controller/agent/graph.py`
  - `controller/api/tasks.py` (if needed)
- Parallel: Yes.
- Notes: Prefer reason codes + concise human detail.

## Test Strategy

- Validate via existing integration planner-gate tests after this WP.
- Smoke-run affected graph transitions with mock responses to ensure no dead routes.

## Risks & Mitigations

- Risk: Breaking existing conditional edge behavior.
- Mitigation: Keep old routing logic as fallback paths only when guard returns allow.

- Risk: Entry failures incorrectly marked as normal rejections.
- Mitigation: Use explicit reason code namespace for entry validation.

## Review Guidance

Reviewer should verify:
- Every first-class engineer node transition is guarded.
- Non-integration reroute and integration fail-fast both implemented.
- Single-node planner graphs fail-closed with clear diagnostics.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
