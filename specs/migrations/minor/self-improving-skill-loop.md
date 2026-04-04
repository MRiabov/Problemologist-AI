# Self-Improving Skill Loop

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration adds a bounded recursive standalone skill-training loop to the provider-backed backend.

The target architecture is described in [agent-skill.md](../../architecture/agents/agent-skill.md) and [agent-harness.md](../../architecture/agents/agent-harness.md). The change is intentionally small in runtime scope but larger in storage scope: evaluation stays thin, while a standalone `train_skills.py`-style entrypoint replays retained episode bundles and resumes the same session across follow-up turns instead of rebuilding context by hand.

## Problem Statement

Today the operator flow is manual:

1. launch a session,
2. stop it on timeout, early exit, or failed simulation success,
3. ask for self-analysis in a second prompt, and
4. ask for skill updates in a third prompt, then copy the result back into the repo.

That workflow works, but it is repetitive and it fragments conversation context across separate prompts. It also throws away the richer episode bundle that would be useful for downstream training.

The architecture already has the right storage boundaries:

1. canonical `skills/`
2. workspace copies in `.agents/skills/`
3. writable `suggested_skills/`
4. durable run memory in `journal.md`
5. retained run artifacts such as review YAML, validation/simulation outputs, render bundles, prompt snapshots, and `events.jsonl`

What is missing is a dedicated training primitive that can reopen the same session from a retained episode bundle and drive multiple follow-up prompts without making the eval launcher or the eval orchestration core own the replay loop.

## Target State

1. A standalone `train_skills.py`-style CLI owns skill training.
2. `dataset/evals/run_evals.py` remains thin and does not own the replay loop.
3. The training CLI treats session resume as the multi-turn primitive for follow-up turns.
4. The loop uses durable workspace state such as `journal.md` and the retained episode bundle to carry the minimum context needed between turns.
5. The first follow-up turn performs self-analysis.
6. The second follow-up turn drafts or repairs the skill content.
7. Generated skill output is written to `suggested_skills/` first.
8. Promotion into the checked-in `skills/` tree remains a separate reviewable step.
9. The loop is bounded and stays asynchronous relative to the main task, eval run, or training pass.

## Required Work

### 1. Add session-resume support

- Capture the session id from the initial launch.
- Add a resume helper that reuses the same session for follow-up prompts.
- Extract the shared session/bundle orchestration into a reusable backend core so both CLIs can call the same logic.
- Split `evals/logic/runner.py` into smaller reusable modules, following the same internal-helper pattern used by `scripts/internal/`.
- If a `scripts/` home produces a cleaner reuse boundary for the CLIs, prefer that over duplicating orchestration logic, but keep the top-level entrypoints thin either way.
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
- Add `train_skills.py` or equivalent as the standalone replay and skill-training CLI.
- Keep `dataset/evals/materialize_seed_workspace.py` as a workspace helper, not the loop owner.
- Put the orchestration in the backend code that already owns session launch, trace capture, and retained-episode bundle loading.

## Non-Goals

- Do not replace the CLI provider with a new transport.
- Do not create a new canonical skill source.
- Do not auto-promote generated skills into `skills/`.
- Do not make the loop unbounded.
- Do not collapse the training loop back into the eval orchestration core or `dataset/evals/run_evals.py`.
- Do not introduce a dedicated journalling agent or skill agent as a required architecture stage.

## Sequencing

The safe order is:

1. Persist the session id and retained episode bundle from the initial run.
2. Add explicit session resume support for follow-up prompts.
3. Split the `evals/logic/runner.py` core from the training CLI while keeping `dataset/evals/run_evals.py` as the thin shim.
4. Add the self-analysis and skill-drafting turns to the training CLI.
5. Keep skill output staged in `suggested_skills/`.
6. Update docs and integration coverage to exercise the loop end to end.

## Acceptance Criteria

1. A timed-out or failed run can be resumed for self-analysis without losing context.
2. The same session can receive multiple follow-up prompts in sequence.
3. Skill-draft output lands in `suggested_skills/` before any promotion step.
4. The retained episode bundle preserves the short outputs and artifacts needed for downstream training.
5. The loop does not require manual copy/paste of prior context.
6. The canonical skill source remains the checked-in `skills/` tree.

## Migration Checklist

Use this checklist to track the implementation from backend wiring through runtime verification. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Session lifecycle and resumption

- [x] Capture the session id from the initial launch.
- [x] Persist the session id in the existing eval/session metadata so later turns can find it deterministically.
- [x] Add an explicit resume helper that targets a known session id rather than relying on implicit session selection.
- [x] Confirm the resume helper can send at least two follow-up prompts into the same live session.
- [x] Keep the resume path non-interactive so the loop can run unattended.

### Loop triggers and control flow

- [x] Trigger the loop when the primary run times out.
- [x] Trigger the loop when the primary run exits early.
- [x] Trigger the loop when simulation success is not reported.
- [x] Keep the follow-up flow bounded to the self-analysis turn and the skill-drafting turn.
- [x] Fail closed if the session cannot be resumed or if the follow-up prompt cannot be delivered.
- [x] Keep the main eval/task run and the skill-improvement loop asynchronous from one another.

### Context preservation

- [x] Keep `journal.md` as the durable run-memory artifact for the follow-up turns.
- [x] Preserve the minimum workspace state needed for later turns without rebuilding the whole transcript by hand.
- [x] Keep any additional loop-state artifact workspace-local and reviewable if one is introduced.
- [x] Verify that a resumed session still has access to the same workspace-relative files the initial run used.

<!--- Human note: journal.md is actually secondary. It's the: responses of agents to particular questions like "You've failed, analyze yourself - what could you do better next time" that counts more. I've found that yet at least, journal.md doesn't generate meaningful data. This is likely because we never say how or what to write in there; the agents just output a minimal safe contract, which we can't use.-->

### Skill-output staging

- [x] Write the self-analysis output into the follow-up flow before generating skill changes.
- [x] Write generated skill content into `suggested_skills/` first.
- [x] Keep canonical `skills/` as the reviewed source of truth.
- [x] Preserve the existing rule that skill updates are explicit, versioned, and reviewable.
- [x] Keep promotion into `skills/` as a separate step, not an automatic side effect of the loop.

### Prompt and artifact wiring

- [x] Add or update reusable prompt fragments for the self-analysis and skill-drafting turns if the backend needs shared prompt text.
- [x] Keep the follow-up prompts short and file-driven instead of embedding the whole history again.
- [x] Ensure the workspace prompt and runtime docs describe the resume-based multi-turn contract clearly enough for provider-backed runs.
- [x] Expose an opt-in CLI flag in the eval entrypoint so the backend loop stays disabled by default.
- [x] Keep `dataset/evals/run_evals.py` thin so it remains a CLI entrypoint, not the loop owner.
- [x] Keep `dataset/evals/materialize_seed_workspace.py` as a workspace helper, not the orchestration layer.
- [x] Add the dedicated `train_skills.py` entrypoint and wire it to retained episode bundles.

### Runner decomposition

- [x] Split `evals/logic/runner.py` into smaller reusable modules without changing the public `dataset/evals/run_evals.py` shim.

### Validation and rollout

- [x] Add integration coverage for a resumed session that receives multiple follow-up prompts.
- [x] Add integration coverage for the timeout/early-exit/no-success triggers that start the loop.
- [x] Add integration coverage that verifies `suggested_skills/` receives the draft before any promotion step.
- [x] Verify the narrowest backend slice first, then widen only if the loop behavior is stable.
- [x] Update any related architecture docs or prompts that still describe manual copy/paste as the only way to continue the conversation.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `evals/logic/codex_workspace.py`
- `evals/logic/codex_session_trace.py`
- `evals/logic/runner.py` and its new helper modules under `evals/logic/`
- `dataset/evals/materialize_seed_workspace.py`
- `dataset/evals/run_evals.py`
- `dataset/evals/train_skills.py`
- `evals/logic/skill_training.py`
- `specs/architecture/agents/agent-skill.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/observability.md`
- `specs/architecture/evals-architecture.md`
- `config/prompts.yaml` if the follow-up prompts need new reusable fragments
- integration tests for the backend and skill-learning path

## Follow-on Refinement

The skill-output staging contract is refined further in [skill-worktree-promotion-arbiter.md](./skill-worktree-promotion-arbiter.md). That follow-on migration treats `suggested_skills/` as a session-local worktree/checkpoint and routes publication into canonical `skills/` through a separate promotion arbiter.
