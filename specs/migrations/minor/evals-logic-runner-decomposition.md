# Evals Logic Runner Decomposition

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration splits `evals/logic/runner.py` into smaller reusable modules under `evals/logic/` while preserving the public eval contract.

The target architecture is described in `specs/devtools.md`, `specs/architecture/evals-architecture.md`, and `specs/architecture/agents/agent-harness.md`. The change is structural rather than behavioral: the public CLI surface, log layout, lock semantics, and eval result semantics should stay stable while the implementation is decomposed. The shape should mirror the repo's `scripts/internal/` pattern: thin public orchestration entrypoints with smaller internal helpers underneath.

## Problem Statement

`evals/logic/runner.py` is currently a 4.5k-line orchestration file. It combines unrelated responsibilities:

01. human-readable logging and pointer generation
02. trace normalization and session metadata writes
03. Codex self-improving skill-loop prompts and event persistence
04. reward-config loading and metric extraction
05. hard-check and judge-check scoring
06. reviewer-chain orchestration and completion-contract checks
07. controller and worker readiness checks
08. git-mode eval execution
09. Codex-mode eval execution
10. CLI parsing, bootstrap, lock handling, and final reporting

That density makes the file hard to evolve and obscures reuse seams for future tooling such as the standalone skill-training loop. The next refactor should carve the runner into focused modules instead of adding more conditionals to the current monolith.

## Current Seam Map

`runner.py` already depends on lower-level helpers that should stay separate:

1. `evals/logic/codex_workspace.py`
2. `evals/logic/codex_session_trace.py`
3. `evals/logic/workspace.py`
4. `evals/logic/review_checks.py`
5. `evals/logic/startup_checks.py`
6. `evals/logic/stack_profiles.py`
7. `evals/logic/dataset_selection.py`
8. `evals/logic/cli_args.py`
9. `scripts/internal/eval_run_lock.py`

The decomposition should move the remaining runner-specific logic into new modules, not duplicate logic that already has a home.

## Target State

1. `evals/logic/runner.py` remains the public eval orchestration façade.
2. The heavy helper logic moves into smaller purpose-specific modules under `evals/logic/`.
3. `dataset/evals/run_evals.py` stays a thin compatibility shim.
4. The standalone `train_skills.py` path can reuse runner helpers later without importing a 4.5k-line module.
5. The CLI contract, log filenames, session metadata fields, and run-lock behavior remain unchanged.

## Facade Cleanup

The decomposition should not freeze a permanent shim stack in place.

1. `dataset/evals/run_evals.py` is temporary compatibility scaffolding for historical invocations, not a long-term ownership layer.
2. `evals/logic/runner.py` should not devolve into a second pass-through shim after the helper extraction is complete.
3. New entrypoints such as `train_skills.py` should import the shared helpers directly and own their workflow, rather than adding another façade over the façade chain.
4. Once call sites have moved, redundant compatibility wrappers should be removed instead of preserved indefinitely.

## Proposed Module Boundaries

| Module | Responsibility | Representative functions |
| -- | -- | -- |
| `evals/logic/runner_reporting.py` | Human-readable log formatting, path rendering, pointer extraction, and session metadata writes | `_console_message`, `_eval_case_label`, `_emit_startup_log_pointers`, `_truncate_text`, `_sanitize_readable_text`, `_format_readable_trace_line`, `_resolve_eval_log_key`, `_append_readable_log_line`, `_mirror_codex_session_trace_to_readable_logs`, `_write_eval_session_metadata`, `_append_jsonl_record` |
| `evals/logic/runner_skill_loop.py` | Codex self-improvement prompts, follow-up turn gating, and skill-loop event persistence | `CodexSkillLoopTurn`, `CodexSkillLoopSummary`, `_load_codex_skill_loop_prompt`, `_codex_skill_loop_needed`, `_codex_skill_loop_prompt_context`, `_build_codex_skill_loop_prompt`, `_write_codex_skill_loop_prompt`, `_record_codex_skill_loop_event`, `_run_codex_skill_loop` |
| `evals/logic/runner_metrics.py` | Reward config loading, metric extraction, hard-check scoring, judge-check scoring, and workspace metric assembly | `_handle_electronics_metrics`, `_load_agent_reward_configs`, `_extract_episode_events`, `_extract_episode_cost_usd`, `_extract_episode_failure_reason`, `_score_milestone_check`, `_codex_workspace_metrics`, `_collect_metrics_for_checks`, `_record_hard_check_outcomes`, `_record_judge_outcomes` |
| `evals/logic/runner_judging.py` | Controller readiness, reviewer-chain execution, episode fetch/interrupt helpers, expected-decision validation, and completion-contract checks | `_planned_counts_as_success`, `_validate_unit_eval_allowlist`, `_wait_for_controller_ready`, `_wait_for_worker_ready`, `_fetch_episode`, `_request_episode_interrupt`, `_missing_required_traces`, `_run_reviewer_chain_for_judge`, `_run_codex_reviewer_chain_for_judge`, `_completion_contract_error` |
| `evals/logic/runner_execution.py` | Git-mode and Codex-mode eval execution plus the single-eval dispatcher | `_run_git_eval`, `_run_codex_eval`, `run_single_eval` |
| `evals/logic/runner.py` | CLI bootstrap, lock acquisition, dataset loading, concurrency, top-level dispatch, and final reporting | `_build_parser`, `main`, `run_cli` |

The exact filenames can shift if an existing helper module is a better fit, but the responsibility split should stay this narrow.

## Non-Goals

- Do not change the public `dataset/evals/run_evals.py` shim contract.
- Do not move the eval launcher into `scripts/` as part of this migration.
- Do not change log directory names, session metadata keys, or eval result semantics.
- Do not create a second eval runner implementation.
- Do not introduce unit tests or mock-only verification for the split.
- Do not fold the standalone skill-training loop back into the monolith.
- Do not add a new permanent facade layer on top of the existing shim chain.

## Sequencing

The safe order is:

1. Extract pure reporting and trace-formatting helpers.
2. Extract the Codex skill-loop helpers and event writers.
3. Extract metric extraction and reward scoring.
4. Extract reviewer-chain and completion-contract helpers.
5. Extract the git and Codex execution paths.
6. Reduce `runner.py` to a thin orchestration shell.
7. Update docs and integration coverage only if the split changes import paths or observable output.

## Acceptance Criteria

1. `evals/logic/runner.py` is materially smaller and acts primarily as orchestration.
2. The new helper modules are individually readable and have a single primary responsibility.
3. `dataset/evals/run_evals.py` still works as the historical entrypoint.
4. The Codex skill-loop path still emits the same durable artifacts and event sidecar.
5. The controller-backed and Codex-backed eval paths still produce the same observable run artifacts and metadata fields.
6. The standalone skill-training loop can reuse the extracted helpers without importing the full runner monolith.

## Migration Checklist

Use this checklist to track the implementation from the first extraction through runtime verification. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Reporting and trace formatting

- [ ] Move human-readable log formatting and pointer generation into a focused helper module.
- [ ] Keep session metadata writes and readable trace rendering stable.
- [ ] Preserve the existing `logs/evals/` file layout and session-log pointers.

### Skill-loop extraction

- [ ] Move Codex self-improvement prompt construction and event recording into a dedicated helper module.
- [ ] Keep the skill-loop trigger semantics unchanged.
- [ ] Preserve the `logs/skill_loop/events.jsonl` sidecar contract.

### Metrics and judging extraction

- [ ] Move reward-config loading, hard-check scoring, and judge-check scoring into a dedicated helper module.
- [ ] Move reviewer-chain orchestration and completion-contract checks into a dedicated helper module.
- [ ] Keep the output schema for hard-check and judge-check reports unchanged.

### Execution extraction

- [ ] Move git-mode eval execution into a dedicated helper module.
- [ ] Move Codex-mode eval execution into a dedicated helper module.
- [ ] Keep `run_single_eval` as the public dispatcher over the extracted execution helpers.

### Runner shell reduction

- [ ] Reduce `evals/logic/runner.py` to parser/bootstrap/dispatch/reporting glue.
- [ ] Keep the `dataset/evals/run_evals.py` compatibility shim unchanged.
- [ ] Confirm the standalone skill-training path can reuse the extracted helpers.
- [ ] Plan the eventual removal of redundant compatibility wrappers once direct call sites are updated.

### Validation and rollout

- [ ] Verify the narrowest Codex/backend slice first, then widen only if the split is stable.
- [ ] Run the relevant eval integration coverage through `./scripts/run_integration_tests.sh`.
- [ ] Update docs only if import paths or observable file contracts change.
- [ ] Retire any wrapper that only forwards to another wrapper once the migration no longer needs it.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `evals/logic/runner.py`
- `evals/logic/runner_reporting.py`
- `evals/logic/runner_skill_loop.py`
- `evals/logic/runner_metrics.py`
- `evals/logic/runner_judging.py`
- `evals/logic/runner_execution.py`
- `dataset/evals/run_evals.py` if the façade needs import adjustments
- `specs/devtools.md`
- `specs/architecture/evals-architecture.md`
- integration tests that cover the Codex runner and the controller-backed eval paths
