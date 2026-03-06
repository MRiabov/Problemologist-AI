---
work_package_id: WP05
title: Observability & Failure Surfacing
lane: "doing"
dependencies: [WP02, WP03, WP04]
base_branch: 012-automatic-node-entry-validation-WP04
base_commit: 06cf43faba071189eb9e8798e37eeafb62ac5f21
created_at: '2026-03-06T11:07:07.593102+00:00'
subtasks:
- T023
- T024
- T025
- T026
- T027
- T028
phase: Phase 4 - Telemetry and Persistence
assignee: ''
agent: "codex"
shell_pid: "41663"
review_status: has_feedback
reviewed_by: MRiabov
review_feedback: feedback://012-automatic-node-entry-validation/WP05/20260306T114153Z-3f0963bd.md
history:
- timestamp: '2026-03-06T08:36:05Z'
  lane: planned
  agent: system
  shell_pid: ''
  action: Prompt generated via /spec-kitty.tasks
requirement_refs:
- FR-009
- NFR-003
- NFR-005
- C-002
- C-004
---

# Work Package Prompt: WP05 - Observability & Failure Surfacing

## Objective

Ensure every node-entry rejection is visible through existing observability boundaries with stable payload shape and deterministic emission ordering.

## Implementation Command

```bash
spec-kitty implement WP05 --base WP04
```

## Context & Constraints

- Observability surfaces:
  - DB traces (`TraceType.LOG/ERROR/EVENT`)
  - Episode metadata (`validation_logs`, status transitions)
  - `events.jsonl` event stream when applicable
- Must align with integration assertions and strict backend log expectations.
- No fallback logging-only path; rejections must be persisted in structured form.

## Subtasks & Detailed Guidance

### T023 - Add or bind entry-validation observability event schema
- Purpose: Standardize event payload structure for entry rejection.
- Steps:
  1. Add new event schema or map to existing event family in `shared/observability/schemas.py`.
  2. Include required fields (`node`, `disposition`, `reason_code`, `errors`, `reroute_target`).
  3. Ensure schema is serializable and stable.
- Files:
  - `shared/observability/schemas.py`
  - possibly `shared/observability/events.py` integration points
- Parallel: No.
- Notes: Prefer extension over ad-hoc dict payloads.

### T024 - Emit structured trace/error payloads on rejection
- Purpose: Make failures observable via episode API trace list.
- Steps:
  1. Emit trace with dedicated name (for example `node_entry_validation_failed`).
  2. Include metadata payload aligned with contract doc.
  3. Ensure emission happens exactly once per rejection event.
- Files:
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: No.
- Notes: Do not double-emit from both guard and downstream error handlers.

### T025 - Propagate reason codes to episode metadata validation logs
- Purpose: Expose machine-assertable failure context in status polling path.
- Steps:
  1. Append reason-code-centric messages into metadata validation logs.
  2. Keep wording stable for integration assertions.
  3. Ensure both engineer and benchmark persistence paths include this behavior.
- Files:
  - `controller/api/tasks.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Preserve existing log entries; add, do not overwrite.

### T026 - Include fail-fast metadata in status broadcasts
- Purpose: Support UI/API consumers and tests that poll status transitions.
- Steps:
  1. Ensure failure broadcasts include relevant metadata vars for entry rejection.
  2. Validate both direct run and continuation paths.
  3. Keep payload backward-compatible.
- Files:
  - `controller/api/tasks.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: Yes.
- Notes: Avoid adding large payloads to broadcast bodies.

### T027 - Add structured logging keys for debugging
- Purpose: Keep backend logs actionable under strict log checks.
- Steps:
  1. Log rejection with structured keys: `node`, `disposition`, `reason_code`, `episode_id`.
  2. Use error-level for true contract failures.
  3. Avoid warning-level downgrade for fail-closed bypass signals.
- Files:
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
  - `controller/api/tasks.py`
- Parallel: Yes.
- Notes: Match strict-backend-errors policy in integration test spec.

### T028 - Enforce deterministic emission order
- Purpose: Prevent flaky integration assertions due to event ordering drift.
- Steps:
  1. Define explicit ordering (entry rejection trace/event before final failed status broadcast).
  2. Verify ordering in both engineer and benchmark flows.
  3. Add guard comments if needed to protect sequencing.
- Files:
  - `controller/agent/graph.py`
  - `controller/agent/benchmark/graph.py`
- Parallel: No.
- Notes: Determinism is essential for integration contract tests.

## Test Strategy

- Validate via integration tests in WP06; manual smoke check that episode traces include the new payload shape.

## Risks & Mitigations

- Risk: Overlapping emissions produce duplicate assertion failures.
- Mitigation: Centralize emission in guard rejection path.

## Review Guidance

Reviewer should verify:
- All required metadata fields are present and stable.
- Emission ordering is deterministic.
- Error-level logs are used for contract failures.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
- 2026-03-06T11:19:57Z – unknown – shell_pid=8707 – lane=for_review – Ready for review: added structured node-entry rejection events/traces and fail-fast metadata propagation
- 2026-03-06T11:37:43Z – codex – shell_pid=41663 – lane=doing – Started review via workflow command
- 2026-03-06T11:41:53Z – codex – shell_pid=41663 – lane=planned – Moved to planned
- 2026-03-06T11:56:44Z – codex – shell_pid=41663 – lane=for_review – Addressed review blocker: restored custom-check lambda contract keyword compatibility. Re-ran tests/integration/architecture_p1/test_handover.py -k benchmark_to_engineer_handoff; previous contract-signature failure is gone (now fails later with reviewer_entry_blocked/connection issue).
- 2026-03-06T11:57:40Z – codex – shell_pid=41663 – lane=doing – Started review via workflow command
