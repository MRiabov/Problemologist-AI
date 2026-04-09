---
title: Global Bug Report Mode
status: completed
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_plan_reviewer
  - engineer_coder
  - engineer_execution_reviewer
  - electronics_planner
  - electronics_reviewer
  - cots_search
  - journalling_agent
  - skill_agent
added_at: '2026-04-09T17:20:11Z'
---

# Global Bug Report Mode

<!-- Implemented. Behavior change and verification landed. -->

## Purpose

This migration adds a single global bug-report mode for prompt-bearing agent
roles. When the mode is enabled through a top-level `bug_reports` block in
`config/agents_config.yaml`, PromptManager appends a short conditional
`bug_reporting` appendix from `config/prompts.yaml` to every active role
prompt. The appendix tells the agent to write `bug_report.md` when the
blocker is caused by infrastructure, harness, workspace materialization,
prompt transport, filesystem policy, render plumbing, eval orchestration, or
other runtime issues rather than by the task itself. Filing the report is
diagnostic, not terminal: the default behavior is to keep working on the
current task after the report is written unless the task is otherwise blocked
or the run has genuinely reached a hard stop.

The target contract lives in
[prompt-management.md](../../architecture/agents/prompt-management.md),
[agent-harness.md](../../architecture/agents/agent-harness.md),
[artifacts-and-filesystem.md](../../architecture/agents/artifacts-and-filesystem.md),
and [devtools.md](../../devtools.md). `bug_report.md` is a maintainer-facing
debug artifact, not a substitute for `journal.md`, `plan_refusal.md`, or any
review artifact.

## Problem Statement

The current workspace contract has no first-class bug-report artifact.
Agents can document blockers in `journal.md`, but that file is task-facing and
does not give maintainers a predictable, dedicated report to harvest when the
runtime itself is the failure source.

That gap creates a few concrete problems:

1. Bug-report instructions are absent from the unified prompt source, so the
   agent never gets a standard, global instruction for infra failures.
2. There is no config-driven toggle that turns bug-report mode on for every
   prompt-bearing role.
3. The filesystem contract does not explicitly allow `bug_report.md` as a
   workspace-root write target across the active agent roles.
4. Devtools have archive paths for integration results and eval logs, but no
   dedicated maintainer-facing collection path for bug reports.
5. The artifact library does not define what a valid `bug_report.md` must
   contain, so later validation would have to rediscover the contract from
   prompts and runtime code.

The result is that overnight debugging loops rely on journals, ad hoc traces,
and manual inspection instead of a clear bug-report file with a stable
collection path.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `config/prompts.yaml` | Has shared, drafting, and backend appendices, but no global `bug_reporting` appendix. | The bug-report instruction should live in the unified prompt source so every role sees the same conditional guidance. |
| `config/agents_config.yaml` | Has no global `bug_reports.enabled` toggle or archive-root setting. | The prompt append, write permission, and maintainer archive path need one source of truth. |
| `controller/agent/prompt_manager.py` | Renders shared, drafting, and backend appendices, but not a global bug-report appendix. | The assembly order needs to include the new conditional appendix. |
| `evals/logic/codex_workspace.py` | Materializes the CLI-provider workspace prompt, but does not inject a global bug-report appendix. | The CLI workspace path must stay aligned with the unified prompt contract. |
| `shared/agents/config.py` | Loads the existing agent config schema, but has no typed bug-report block. | The runtime needs a typed config source for the new global toggle. |
| `shared/workers/filesystem/policy.py` | Enforces the current path allowlist, but does not know about `bug_report.md`. | The filesystem contract must explicitly allow the new workspace-root report file. |
| `specs/architecture/agents/agent-harness.md` | Describes the workspace prompt and submission contract, but not `bug_report.md`. | The harness needs to say when the file should be written and when the mode is active. |
| `specs/architecture/agents/artifacts-and-filesystem.md` | Enumerates current role, review manifests, and render ownership, but not a bug-report artifact. | The filesystem contract must explicitly allow the new workspace-root report file. |
| `specs/architecture/agents/agent-artifacts/README.md` | Has file-level acceptance criteria for existing artifacts, but no bug-report entry. | The new artifact needs a contract entry so later validation can be strict. |
| `specs/architecture/agents/roles.md` and `specs/architecture/agents/roles-detailed/` | Role guidance does not tell agents when to file a bug report instead of continuing normal task work. | The prompt-facing role guidance must tell the agent how to use the new mode. |
| `specs/devtools.md` | Documents integration and eval log persistence, but no bug-report archive helper or archive tree. | Maintainers need a stable collection path for run-time bug reports. |
| `specs/architecture/observability.md` | Tracks many debug and review events, but not a dedicated bug-report persistence surface. | The bug-report archive should be queryable or at least traceable alongside the rest of the run metadata. |

## Proposed Target State

1. Bug-report mode is controlled by one global `bug_reports.enabled` config
   flag in `config/agents_config.yaml`, not by per-role opt-in flags.
2. When the flag is enabled, PromptManager appends the same bug-report
   appendix to every prompt-bearing role, including helper roles and
   reviewers that can write workspace artifacts.
3. The bug-report appendix tells the agent to write `bug_report.md` only
   when the failure is caused by infrastructure, harness, workspace
   materialization, prompt transport, filesystem policy, render plumbing,
   eval orchestration, or other runtime plumbing.
4. `bug_report.md` is a workspace-root Markdown artifact owned by the active
   role while bug-report mode is enabled, and the same mode toggle adds the
   path to the role's write allowlist.
5. The report stays concise and factual. It names the active role, the
   current session or episode identifiers when available, the exact failing
   command or tool call, the expected versus actual behavior, and the
   relevant artifact paths.
6. Writing `bug_report.md` does not mean the agent should stop by default.
   The agent keeps working after the report is filed unless the current task
   is actually blocked or the run has reached a hard stop.
7. `bug_report.md` is separate from `journal.md`, `plan_refusal.md`, and the
   stage review YAML files. It does not replace them and does not alter the
   normal handoff or review routing.
8. Devtools own the maintainer-facing collection path. A public persistence
   helper snapshots `bug_report.md` into a dedicated run archive tree under
   `logs/bug_reports/` with run/session metadata so overnight loops can be
   triaged without manual workspace scavenging.
9. The feature fails closed. If the mode is off, the appendix is absent. If
   the file is absent, no archive entry is produced. The runtime does not
   infer a bug-report contract from filename shape, prompt text, or workspace
   content alone.

## Required Work

### 1. Add the global bug-report config surface

- Add a top-level `bug_reports` configuration block to
  `config/agents_config.yaml`.
- Use one global enable switch and one maintainer archive root, rather than
  per-role bug-report flags.
- Make the prompt-injection decision and the write-permission decision read
  from the same config source.
- Add the matching config schema in `shared/agents/config.py`.

### 2. Add the conditional prompt appendix

- Add a `bug_reporting` appendix to `config/prompts.yaml`.
- Route the appendix through the same prompt assembly path that already owns
  the shared, drafting, and backend appendices.
- Keep the appendix short and role-agnostic so it applies uniformly to every
  prompt-bearing agent.
- State explicitly that filing `bug_report.md` is a diagnostic action, not a
  stop condition, and that the agent should continue working unless the task
  is otherwise blocked.
- Update `controller/agent/prompt_manager.py` and
  `evals/logic/codex_workspace.py` so the same appendix is visible in both
  controller-backed and CLI-provider-backed prompt materialization.

### 3. Define the artifact contract

- Add `specs/architecture/agents/agent-artifacts/bug_report_md_acceptance_criteria.md`.
- Register `bug_report.md` in `specs/architecture/agents/agent-artifacts/README.md`.
- Update the filesystem policy code in `shared/workers/filesystem/policy.py`
  so `bug_report.md` is described as a workspace-root debug artifact with
  explicit write permissions when the mode is enabled.
- Update the filesystem and harness docs so `bug_report.md` is described as a
  workspace-root debug artifact with explicit write permissions when the mode
  is enabled.

### 4. Update role guidance

- Update `specs/architecture/agents/roles.md` so the cross-role contract names
  bug-report mode explicitly.
- Update the detailed role sheets under
  `specs/architecture/agents/roles-detailed/` so they tell each role when to
  file `bug_report.md` and when to keep using `journal.md` instead.
- Keep the guidance conditional on actual infrastructure or runtime blockers.

### 5. Add maintainer-facing archive plumbing

- Add a public devtool such as `scripts/persist_bug_reports.py` that copies
  `bug_report.md` into a run-scoped archive tree under `logs/bug_reports/`.
- Persist enough metadata for maintainers to tie the report back to the
  current role, session or episode, and workspace path.
- Update `specs/devtools.md` so the bug-report archive path becomes part of
  the documented developer instrumentation surface.

### 6. Refresh observability and validation

- Update `specs/architecture/observability.md` if the archive path or bug
  report metadata needs to be queryable as a run artifact.
- Add integration coverage for prompt materialization, workspace write
  permission, and bug-report archival.
- Verify the narrow prompt and workspace slice first before widening to the
  archive helper.

## Non-Goals

- Do not make bug reports mandatory on every turn.
- Do not make `bug_report.md` a default stop signal.
- Do not turn `bug_report.md` into a replacement for `journal.md`,
  `plan_refusal.md`, or review artifacts.
- Do not create per-agent bug-report modes or role-specific report formats.
- Do not add a new tool endpoint just to write the file.
- Do not change the core handoff graph, planner submission gates, or review
  routing.
- Do not broaden filesystem access beyond the minimum required to write and
  archive the report.

## Sequencing

The safe order is:

1. Define the artifact contract and the global config toggle.
2. Add the conditional prompt appendix and the matching workspace write
   permission.
3. Update the role guidance and the artifact index docs.
4. Add the maintainer-facing archive helper and its run metadata.
5. Refresh observability notes and integration coverage.

## Acceptance Criteria

1. Enabling the global bug-report flag causes every prompt-bearing role to
   receive the bug-report appendix.
2. The workspace contract allows enabled roles to write `bug_report.md` and
   rejects the file when bug-report mode is off.
3. The artifact contract describes `bug_report.md` as a stable,
   maintainer-facing debug file with concrete content requirements.
4. Devtools can collect `bug_report.md` into a run archive tree without
   confusing it with journal, review, or test-history artifacts.
5. Filing `bug_report.md` does not stop the agent by default; the prompt
   contract keeps the agent working unless the current task is genuinely
   blocked.
6. The system remains fail-closed: missing or malformed bug-report state does
   not invent a fallback contract.

## Migration Checklist

### Config and prompt surface

- [x] Add a typed `bug_reports` config block to `shared/agents/config.py`.
- [x] Add the global bug-report config values to `config/agents_config.yaml`.
- [x] Add a `bug_reporting` appendix to `config/prompts.yaml`.
- [x] Update `controller/agent/prompt_manager.py` so the appendix is appended
  when bug-report mode is enabled and omitted when it is disabled.
- [x] Update `evals/logic/codex_workspace.py` so CLI-provider prompt
  materialization sees the same conditional appendix.
- [x] Verify that the bug-report appendix stays global and does not depend on
  agent role name, backend family, or technical-drawing mode.

### Artifact contract and permissions

- [x] Add `bug_report.md` to the artifact contract library under
  `specs/architecture/agents/agent-artifacts/`.
- [x] Register `bug_report.md` in
  `specs/architecture/agents/agent-artifacts/README.md`.
- [x] Update `specs/architecture/agents/artifacts-and-filesystem.md` so the
  workspace-root report file is explicitly owned by the active role when bug
  report mode is enabled.
- [x] Update `specs/architecture/agents/agent-harness.md` so the workspace
  contract tells the agent to keep working after filing a bug report by
  default.
- [x] Update `specs/architecture/agents/roles.md` and the relevant detailed
  role sheets so the filing rule is explicit and conditional on infra/runtime
  blockers.
- [x] Add the `bug_report.md` write allowlist entry to
  `shared/workers/filesystem/policy.py` behind the same global config toggle.

### Devtools and observability

- [x] Add `scripts/persist_bug_reports.py` or an equivalent public archive
  helper.
- [x] Make the archive helper copy `bug_report.md` into a run-scoped tree
  under `logs/bug_reports/`.
- [x] Persist the role, episode or session identifiers, workspace path, and
  archived report path with each archived bug report.
- [x] Update `specs/devtools.md` so the bug-report archive tree is part of
  the developer instrumentation surface.
- [x] Update `specs/architecture/observability.md` if the archive metadata
  needs to be queryable alongside the other run artifacts.
- [x] Keep the archive helper separate from `journal.md`, review YAML, and
  test-history persistence.

### Validation

- [x] Add integration coverage for prompt materialization when bug-report
  mode is enabled and when it is disabled.
- [x] Add integration coverage for workspace write permission to
  `bug_report.md`.
- [x] Add integration coverage for the archive helper copying a report with
  metadata.
- [x] Add a regression that writing `bug_report.md` does not stop the agent
  by default.
- [x] Verify the narrow prompt/workspace slice before widening to archive
  and observability coverage.

## File-Level Change Set

The implementation should touch the smallest realistic set of files that
enforce the new contract:

- `config/agents_config.yaml`
- `config/prompts.yaml`
- `controller/agent/prompt_manager.py`
- `evals/logic/codex_workspace.py`
- `shared/agents/config.py`
- `shared/workers/filesystem/policy.py`
- `specs/architecture/agents/prompt-management.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/agent-artifacts/README.md`
- `specs/architecture/agents/agent-artifacts/bug_report_md_acceptance_criteria.md`
- `specs/architecture/agents/roles.md`
- `specs/architecture/agents/roles-detailed/*.md`
- `specs/devtools.md`
- `specs/architecture/observability.md` if queryable archive metadata is
  added
- `scripts/persist_bug_reports.py` or an equivalent public archive helper
- integration tests for prompt materialization and bug-report persistence

## Open Questions

1. Should the bug-report archive live under `logs/bug_reports/` as a new
   public tree, or should it be folded into an existing run-log family?
2. Should reviewer roles be allowed to write `bug_report.md` whenever the
   mode is enabled, or should the mode be limited to non-reviewer roles only?
3. Should the archive helper persist only the latest `bug_report.md` per run,
   or should it snapshot updates when the file changes multiple times during
   the same run?
