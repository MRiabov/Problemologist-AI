---
work_package_id: WP07
title: Compatibility, Regression, and Documentation Sync
lane: "doing"
dependencies: [WP06]
base_branch: 012-automatic-node-entry-validation-WP06
base_commit: 14e61b8a93abe28ccf6467517103b161182f1402
created_at: '2026-03-06T12:43:14.885420+00:00'
subtasks:
- T036
- T037
- T038
- T039
- T040
phase: Phase 6 - Hardening
assignee: ''
agent: "codex"
shell_pid: "41663"
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
- FR-010
- NFR-005
- C-004
- C-005
---

# Work Package Prompt: WP07 - Compatibility, Regression, and Documentation Sync

## Objective

Stabilize the feature after integration coverage lands: preserve existing contracts, resolve regressions, and synchronize implementation-facing docs.

## Implementation Command

```bash
spec-kitty implement WP07 --base WP06
```

## Context & Constraints

- This WP is polish/hardening, not net-new architecture.
- Must preserve existing planner/reviewer gate behavior and policy constraints.
- Keep reviewer write scope and `.manifests/**` rules unchanged.

## Subtasks & Detailed Guidance

### T036 - Execute targeted and baseline integration suites
- Purpose: Validate no regressions in adjacent architecture contracts.
- Steps:
  1. Run new node-entry integration file.
  2. Run planner-gate and reviewer-handover related integration files.
  3. Run integration P0 marker suite and collect failures.
- Files:
  - test runner artifacts/logs only
- Parallel: No.
- Notes: Capture exact failing tests and error signatures.

### T037 - Resolve regressions introduced by guard integration
- Purpose: Restore baseline while keeping new guard contracts.
- Steps:
  1. Fix failing code paths caused by guard insertion.
  2. Ensure fixes do not weaken fail-closed policy.
  3. Re-run impacted tests.
- Files:
  - whichever runtime files fail under T036
- Parallel: No.
- Notes: Prefer source-fix over fallback paths.

### T038 - Sync quickstart/plan notes with actual behavior
- Purpose: Keep feature docs accurate after implementation details settle.
- Steps:
  1. Update `kitty-specs/012-automatic-node-entry-validation/quickstart.md` if commands/assertions changed.
  2. Update `plan.md` execution/risk notes only if materially different from implemented reality.
  3. Keep changes minimal and factual.
- Files:
  - `kitty-specs/012-automatic-node-entry-validation/quickstart.md`
  - `kitty-specs/012-automatic-node-entry-validation/plan.md` (if needed)
- Parallel: Yes.
- Notes: No speculative edits.

### T039 - Validate manifest/reviewer policy invariants remain unchanged
- Purpose: Prevent accidental security/policy regressions.
- Steps:
  1. Verify `.manifests/**` access policy behavior remains unchanged.
  2. Verify reviewer write/edit scope constraints remain enforced.
  3. Confirm integration assertions for these contracts still pass.
- Files:
  - relevant policy/integration files only if regression found
- Parallel: Yes.
- Notes: This is a must-pass architecture boundary.

### T040 - Add final reviewer-facing implementation notes
- Purpose: Make code review and future maintenance clearer.
- Steps:
  1. Add concise notes in changed files and/or feature docs summarizing expected entry-failure signatures.
  2. Include reason-code and disposition examples.
  3. Ensure notes are actionable for future debugging.
- Files:
  - `kitty-specs/012-automatic-node-entry-validation/quickstart.md`
  - minimal inline comments in key orchestration files (if useful)
- Parallel: Yes.
- Notes: Keep noise low; clarity high.

## Test Strategy

Required:

```bash
./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_node_entry_validation.py
./scripts/run_integration_tests.sh -m integration_p0
```

## Risks & Mitigations

- Risk: Attempting to fix too many unrelated pre-existing failures.
- Mitigation: Isolate regressions caused by this feature and document external blockers explicitly.

## Review Guidance

Reviewer should verify:
- New behavior is stable.
- Existing critical contracts remain enforced.
- Documentation reflects the implemented contract and commands.

## Activity Log

- 2026-03-06T08:36:05Z - system - lane=planned - Prompt created.
- 2026-03-06T12:43:15Z – codex – shell_pid=8707 – lane=doing – Assigned agent via workflow command
- 2026-03-06T13:08:36Z – codex – shell_pid=8707 – lane=for_review – Ready for review: expected guard failure signatures annotated in planner integration tests; required node-entry suite passes; integration_p0 has unrelated baseline failure INT-020 (simulation taxonomy).
- 2026-03-06T13:12:15Z – codex – shell_pid=41663 – lane=doing – Started review via workflow command
