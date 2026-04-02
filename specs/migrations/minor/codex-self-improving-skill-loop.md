# Codex Self-Improving Skill Loop

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration adds a bounded recursive skill-improvement loop to the Codex backend.

The target architecture is described in [agent-skill.md](../../architecture/agents/agent-skill.md) and [agent-harness.md](../../architecture/agents/agent-harness.md). The change is intentionally small: it keeps the existing skill ownership model intact and adds a way to resume the same Codex session across follow-up turns instead of rebuilding context by hand.

## Problem Statement

Today the operator flow is manual:

1. launch a Codex session,
2. stop it on timeout, early exit, or failed simulation success,
3. ask for self-analysis in a second prompt, and
4. ask for skill updates in a third prompt, then copy the result back into the repo.

That workflow works, but it is repetitive and it fragments conversation context across separate prompts.

The architecture already has the right storage boundaries:

1. canonical `skills/`
2. workspace copies in `.agents/skills/`
3. writable `suggested_skills/`
4. durable run memory in `journal.md`

What is missing is a runtime primitive that can keep the same Codex session alive across multiple follow-up prompts.

## Target State

1. The Codex backend treats `codex exec resume <session_id> <prompt>` as the multi-turn primitive for follow-up turns.
2. The runner persists the Codex session id and workspace metadata so later turns can resume deterministically.
3. The loop uses durable workspace state such as `journal.md` to carry the minimum context needed between turns.
4. The first follow-up turn performs self-analysis.
5. The second follow-up turn drafts or repairs the skill content.
6. Generated skill output is written to `suggested_skills/` first.
7. Promotion into the checked-in `skills/` tree remains a separate reviewable step.
8. The loop is bounded and stays asynchronous relative to the main task or eval run.

## Required Work

### 1. Add session-resume support

- Capture the session id from the initial `codex exec` launch.
- Add a resume helper that reuses the same session for follow-up prompts.
- Keep the resume path explicit; do not depend on implicit session selection for automation.

### 2. Add a bounded follow-up sequence

- Trigger the follow-up sequence when the primary run times out, exits early, or fails to report simulation success.
- Resume the session once for self-analysis.
- Resume the session again for skill drafting.
- Keep each follow-up prompt short and file-driven so the loop does not depend on rebuilding the full conversation manually.

### 3. Preserve staged skill output

- Write the skill draft into `suggested_skills/`.
- Keep canonical `skills/` as the reviewed source of truth.
- Preserve the current contract that skill updates are explicit and versioned.

### 4. Wire the orchestration into the eval backend

- Keep `dataset/evals/run_evals.py` as a thin CLI entrypoint.
- Keep `dataset/evals/materialize_seed_workspace.py` as a workspace helper, not the loop owner.
- Put the orchestration in the backend code that already owns Codex session launch and trace capture.

## Non-Goals

- Do not replace `codex exec` with a new transport.
- Do not create a new canonical skill source.
- Do not auto-promote generated skills into `skills/`.
- Do not make the loop unbounded.
- Do not change the existing async separation between the main run and the skill-learning path.

## Sequencing

The safe order is:

1. Persist the Codex session id from the initial run.
2. Add explicit session resume support for follow-up prompts.
3. Add the self-analysis and skill-drafting turns.
4. Keep skill output staged in `suggested_skills/`.
5. Update docs and integration coverage to exercise the loop end to end.

## Acceptance Criteria

1. A timed-out or failed Codex run can be resumed for self-analysis without losing context.
2. The same session can receive multiple follow-up prompts in sequence.
3. Skill-draft output lands in `suggested_skills/` before any promotion step.
4. The loop does not require manual copy/paste of prior context.
5. The canonical skill source remains the checked-in `skills/` tree.

## Migration Checklist

Use this checklist to track the implementation from backend wiring through runtime verification. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Session lifecycle and resumption

- [x] Capture the Codex session id from the initial `codex exec` launch.
- [x] Persist the session id in the existing eval/session metadata so later turns can find it deterministically.
- [x] Add an explicit resume helper that targets a known session id rather than relying on implicit session selection.
- [x] Confirm the resume helper can send at least two follow-up prompts into the same live session.
- [x] Keep the resume path non-interactive so the loop can run unattended.

### Loop triggers and control flow

- [ ] Trigger the loop when the primary run times out.
- [x] Trigger the loop when the primary run exits early.
- [x] Trigger the loop when simulation success is not reported.
- [x] Keep the follow-up flow bounded to the self-analysis turn and the skill-drafting turn.
- [ ] Fail closed if the session cannot be resumed or if the follow-up prompt cannot be delivered.
- [x] Keep the main eval/task run and the skill-improvement loop asynchronous from one another.

### Context preservation

- [ ] Keep `journal.md` as the durable run-memory artifact for the follow-up turns.
- [ ] Preserve the minimum workspace state needed for later turns without rebuilding the whole transcript by hand.
- [ ] Keep any additional loop-state artifact workspace-local and reviewable if one is introduced.
- [ ] Verify that a resumed session still has access to the same workspace-relative files the initial run used.

### Skill-output staging

- [x] Write the self-analysis output into the follow-up flow before generating skill changes.
- [ ] Write generated skill content into `suggested_skills/` first.
- [ ] Keep canonical `skills/` as the reviewed source of truth.
- [ ] Preserve the existing rule that skill updates are explicit, versioned, and reviewable.
- [ ] Keep promotion into `skills/` as a separate step, not an automatic side effect of the loop.

### Prompt and artifact wiring

- [x] Add or update reusable prompt fragments for the self-analysis and skill-drafting turns if the backend needs shared prompt text.
- [x] Keep the follow-up prompts short and file-driven instead of embedding the whole history again.
- [x] Ensure the workspace prompt and runtime docs describe the resume-based multi-turn contract clearly enough for Codex runs.
- [x] Expose an opt-in CLI flag in the Codex eval entrypoint so the backend loop stays disabled by default.
- [ ] Keep `dataset/evals/run_evals.py` thin so it remains a CLI entrypoint, not the loop owner.
- [ ] Keep `dataset/evals/materialize_seed_workspace.py` as a workspace helper, not the orchestration layer.

### Validation and rollout

- [x] Add integration coverage for a resumed Codex session that receives multiple follow-up prompts.
- [ ] Add integration coverage for the timeout/early-exit/no-success triggers that start the loop.
- [ ] Add integration coverage that verifies `suggested_skills/` receives the draft before any promotion step.
- [ ] Verify the narrowest Codex/backend slice first, then widen only if the loop behavior is stable.
- [x] Update any related architecture docs or prompts that still describe manual copy/paste as the only way to continue the conversation.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `evals/logic/codex_workspace.py`
- `evals/logic/codex_session_trace.py`
- `evals/logic/runner.py`
- `dataset/evals/materialize_seed_workspace.py`
- `dataset/evals/run_evals.py`
- `specs/architecture/agents/agent-skill.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/evals-architecture.md`
- `config/prompts.yaml` if the follow-up prompts need new reusable fragments
- integration tests for the Codex backend and skill-learning path
