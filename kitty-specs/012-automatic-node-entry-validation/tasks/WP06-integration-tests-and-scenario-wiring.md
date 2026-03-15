---
work_package_id: WP06
title: Integration Tests & Scenario Wiring
lane: "done"
dependencies: [WP04, WP05]
base_branch: 012-automatic-node-entry-validation-WP06-merge-base
base_commit: 4a1a92d9a378285a450eb8f13ef7a1d6310749a8
created_at: '2026-03-06T12:04:45.089990+00:00'
subtasks:
- T029
- T030
- T031
- T032
- T033
- T034
- T035
phase: Phase 5 - Contract Verification
assignee: ''
agent: codex
shell_pid: '41663'
review_status: "approved"
reviewed_by: "MRiabov"
review_feedback: ''
history:
- timestamp: '2026-03-06T08:36:05Z'
  lane: planned
  agent: system
  shell_pid: ''
  action: Prompt generated via /spec-kitty.tasks
requirement_refs:
- FR-012
- NFR-001
- NFR-003
- C-001
- C-002
---

# Work Package Prompt: WP06 - Integration Tests & Scenario Wiring

## Objective

Deliver integration-only test coverage for node-entry validation contracts (reroute and fail-fast), with assertions against HTTP-visible state, traces, and events.

## Implementation Command

```bash
spec-kitty implement WP06 --base WP05
```

## Context & Constraints

- Integration test policy is strict in this repository:
  - No unit-test substitution.
  - HTTP-driven execution and assertions on observable boundaries.
  - Prefer polling/state-based waits over sleeps.
- Key files:
  - `specs/integration-tests.md`
  - `tests/integration/mock_responses/`
  - `tests/integration/architecture_p0/`

## Subtasks & Detailed Guidance

### T029 - Add integration spec ID mapping
- Purpose: Ensure coverage is tracked in official integration matrix.
- Steps:
  1. Add new `INT-xxx` row under appropriate priority section in `specs/integration-tests.md`.
  2. Define required assertions for entry validation reroute/fail-fast behavior.
  3. Keep wording aligned with fail-closed architecture language.
- Files:
  - `specs/integration-tests.md`
- Parallel: Yes.
- Notes: Keep this ID referenced in new test docstrings.

### T030 - Create integration test module scaffold
- Purpose: Establish dedicated test coverage location.
- Steps:
  1. Create `tests/integration/architecture_p0/test_node_entry_validation.py`.
  2. Add reusable poll helpers and parsing utilities for traces/metadata.
  3. Keep tests deterministic and avoid hardcoded sleeps.
- Files:
  - `tests/integration/architecture_p0/test_node_entry_validation.py` (new)
- Parallel: No.
- Notes: Keep helper logic local unless shared widely.

### T031 - Add mock transcript scenarios for invalid entry
- Purpose: Deterministically trigger entry guard failures in integration mode.
- Steps:
  1. Add scenario keys in `tests/integration/mock_responses/` for entry-failure paths.
  2. Ensure transcripts force transitions into nodes with missing prerequisites.
  3. Keep scenarios minimal and explicit about expected traces.
- Files:
  - `tests/integration/mock_responses/`
- Parallel: Yes.
- Notes: Reuse existing scenario naming conventions.

### T032 - Assert target-node execution is skipped
- Purpose: Validate core “pre-entry block” guarantee.
- Steps:
  1. In tests, assert no execution evidence for target node on rejected turn.
  2. Use trace ordering and node names to prove no target-node invocation.
  3. Cover at least one engineer path and one benchmark path.
- Files:
  - `tests/integration/architecture_p0/test_node_entry_validation.py`
- Parallel: Yes.
- Notes: Avoid brittle assertions tied to incidental message text.

### T033 - Assert integration fail-fast behavior
- Purpose: Guarantee one-turn failure transition in integration mode.
- Steps:
  1. Force invalid entry in integration mode.
  2. Poll episode until terminal status.
  3. Assert `FAILED` without loopback retries and with explicit reason evidence.
- Files:
  - `tests/integration/architecture_p0/test_node_entry_validation.py`
- Parallel: Yes.
- Notes: Use state-based polling only.

### T034 - Assert non-integration reroute behavior
- Purpose: Validate runtime recovery policy where harness allows mode split.
- Steps:
  1. Add test path or mode override that executes non-integration behavior.
  2. Assert reroute to deterministic previous node on entry rejection.
  3. Assert status does not immediately fail from first rejection.
- Files:
  - `tests/integration/architecture_p0/test_node_entry_validation.py`
  - test harness/config files only if required
- Parallel: Yes.
- Notes: If harness cannot run non-integration path, document constraint and cover via nearest observable contract.

### T035 - Assert trace/event payload contract
- Purpose: Lock metadata schema behavior for regression protection.
- Steps:
  1. Assert presence of `node`, `disposition`, `reason_code`, and structured `errors` fields in traces/events.
  2. Assert disposition values match runtime mode (`reroute_previous` vs `fail_fast`).
  3. Assert stable reason-code semantics.
- Files:
  - `tests/integration/architecture_p0/test_node_entry_validation.py`
- Parallel: Yes.
- Notes: Keep assertions schema-first, text-second.

## Test Strategy

Required commands:

```bash
./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_node_entry_validation.py
./scripts/run_integration_tests.sh -m integration_p0
```

If a subset fails due to unrelated existing issues, isolate and report precisely.

## Risks & Mitigations

- Risk: Flaky waits due to async status propagation.
- Mitigation: Poll with deterministic terminal conditions and bounded retries.

- Risk: Overfitting to mock transcript ordering.
- Mitigation: Assert contract fields and states, not incidental prompt text.

## Review Guidance

Reviewer should verify:
- Tests use HTTP-only boundaries and integration markers.
- New `INT-xxx` mapping exists and matches test docstrings.
- Both behavior modes are covered or explicitly justified when harness-limited.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
- 2026-03-06T12:04:45Z – codex – shell_pid=8707 – lane=doing – Assigned agent via workflow command
- 2026-03-06T12:25:28Z – codex – shell_pid=8707 – lane=for_review – Ready for review: INT-184 integration coverage and entry-validation metadata persistence
- 2026-03-06T12:39:12Z – codex – shell_pid=41663 – lane=doing – Started review via workflow command
- 2026-03-06T12:41:49Z – codex – shell_pid=41663 – lane=done – Review passed: INT-184 mapping/tests added; node-entry metadata contract validated. New module tests pass (2/2). Full integration_p0 blocked by unrelated frontend build dependency errors (missing react/react-router-dom/lucide-react type modules). | Done override: Stacked feature workflow: WP06 reviewed on feature branch before merge to main; status reflects review completion.
