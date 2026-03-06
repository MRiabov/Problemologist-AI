---
work_package_id: WP03
title: Benchmark Graph Guard Integration
lane: "doing"
dependencies: [WP01]
base_branch: 012-automatic-node-entry-validation-WP02
base_commit: ae1c7ce385e49f01271ca1e8a7b08caa270915f5
created_at: '2026-03-06T10:10:11.515229+00:00'
subtasks:
- T012
- T013
- T014
- T015
- T016
- T017
phase: Phase 2 - Benchmark Runtime Integration
assignee: ''
agent: "codex"
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
- FR-007
- FR-010
- NFR-001
- NFR-002
- C-003
---

# Work Package Prompt: WP03 - Benchmark Graph Guard Integration

## Objective

Integrate pre-entry validation into benchmark orchestration (`planner -> coder -> reviewer`) including streaming execution and continuation flows.

## Implementation Command

```bash
spec-kitty implement WP03 --base WP01
```

## Context & Constraints

- Core files:
  - `controller/agent/benchmark/graph.py`
  - `controller/agent/benchmark/state.py`
  - `controller/agent/benchmark/nodes.py`
- Existing planner handoff checks and reviewer handoff checks must remain intact.
- New entry guards must run before target node execution, not after.

## Subtasks & Detailed Guidance

### T012 - Add benchmark graph entry-guard routing
- Purpose: Guard every first-class benchmark node transition.
- Steps:
  1. Add guard-aware transition logic in `define_graph()`.
  2. Apply to planner/coder/reviewer and relevant auxiliary nodes in scope.
  3. Ensure rejected entry never invokes target node callable.
- Files:
  - `controller/agent/benchmark/graph.py`
- Parallel: No.
- Notes: Keep parity with engineer graph behavior.

### T013 - Define benchmark node contracts
- Purpose: Encode benchmark-specific preconditions.
- Steps:
  1. Add contract definitions for benchmark planner/coder/reviewer.
  2. Include planner artifact and state prerequisites for coder entry.
  3. Include reviewer handover prerequisites for reviewer entry.
- Files:
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Reuse shared validators where possible.

### T014 - Implement non-integration reroute in benchmark flow
- Purpose: Recoverable behavior for normal runtime.
- Steps:
  1. On rejection, select deterministic previous node target.
  2. Append reason to benchmark `validation_logs` and `review_feedback` where appropriate.
  3. Preserve existing pause/PLANNED semantics when entry succeeds.
- Files:
  - `controller/agent/benchmark/graph.py`
- Parallel: No.
- Notes: Do not regress current planner pause behavior.

### T015 - Implement integration fail-fast in benchmark flow
- Purpose: Prevent integration retry loops.
- Steps:
  1. For integration mode entry rejection, transition to failed state and stop stream progression.
  2. Set terminal reason/failure class in line with current metadata conventions.
  3. Ensure this path is distinguishable from reviewer rejection loops.
- Files:
  - `controller/agent/benchmark/graph.py`
- Parallel: No.
- Notes: Keep one-turn fail behavior deterministic.

### T016 - Guard start/continue paths
- Purpose: Cover resumed episodes and continuation edge cases.
- Steps:
  1. Ensure `run_generation_session` path and route_start decisions cannot bypass entry checks.
  2. Ensure `continue_generation_session` path also applies entry checks before node execution.
  3. Handle invalid resumed state with clear failure/rejection reason.
- Files:
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: This is critical for stale filesystem state scenarios.

### T017 - Persist benchmark failure metadata for entry rejections
- Purpose: Make entry failures visible in benchmark episode persistence.
- Steps:
  1. Propagate reason code/details into metadata and validation logs.
  2. Ensure status broadcast includes proper failed/rejected context.
  3. Keep existing asset sync behavior unaffected by early failure.
- Files:
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Coordinate with WP05 observability schema.

## Test Strategy

- Run benchmark planner/coder/reviewer integration flows with mocked transcripts that force invalid entry.
- Validate no target node execution when guard rejects entry.

## Risks & Mitigations

- Risk: Interference with existing planner pause semantics (`PLANNED`).
- Mitigation: Only apply rejection path on guard failure; preserve existing success path untouched.

- Risk: Duplicate failure handling between coder/reviewer and guard layers.
- Mitigation: Treat guard failure as transition-level event with dedicated reason codes.

## Review Guidance

Reviewer should verify:
- All benchmark transitions are entry-gated.
- Continue-generation path is protected.
- Integration fail-fast is deterministic and non-looping.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
- 2026-03-06T10:10:12Z – codex – shell_pid=8707 – lane=doing – Assigned agent via workflow command
