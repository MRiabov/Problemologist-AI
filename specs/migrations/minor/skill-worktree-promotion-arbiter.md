---
title: Skill Worktree and Promotion Arbiter
status: investigation
agents_affected: []
added_at: '2026-04-04T09:53:43Z'
---

# Skill Worktree and Promotion Arbiter

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This follow-on migration refines the skill-training storage and publication model after the bounded skill-loop exists.

The previous migration taught the runtime how to resume the same session, write skill deltas, and retain enough bundle state for later replay. This migration changes how those deltas are carried between turns and how they become canonical:

1. `suggested_skills/` becomes the active per-run skill worktree/checkpoint for a training session.
2. Later turns read the session overlay first instead of treating it as a dead-end copy.
3. Publication back into the checked-in `skills/` tree is handled by a separate promotion arbiter.

The target architecture is described in [agent-skill.md](../../architecture/agents/agent-skill.md), [agent-harness.md](../../architecture/agents/agent-harness.md), [prompt-management.md](../../architecture/agents/prompt-management.md), [evals-architecture.md](../../architecture/evals-architecture.md), and [observability.md](../../architecture/observability.md).

## Problem Statement

The current training story still has two implicit copies of the skill tree:

1. canonical `skills/`, which is approved and published,
2. `suggested_skills/`, which currently behaves like staging.

That staging model is too weak for recursive improvement. It makes later turns behave as if they must rediscover their own draft state, and it leaves conflict resolution underspecified when multiple training runs produce updates for the same skill.

The desired model is closer to a session-local worktree:

1. the active training run owns its own mutable skill overlay,
2. the overlay is derived from an approved canonical base commit,
3. later turns see their own prior skill edits immediately,
4. independent runs do not share mutable draft state,
5. a separate arbiter decides whether and how the resulting diff gets published into canonical `skills/`.

This separates learning from publication and keeps the training loop from becoming a git/publishing workflow itself.

## Target State

1. `suggested_skills/` is the active skill worktree/checkpoint for one training session.
2. The training CLI resolves the session overlay first, then the approved canonical `skills/` tree.
3. New training runs start from the approved canonical skill snapshot or locked commit, not from another run's in-flight overlay.
4. The training loop may write into `suggested_skills/`, but it does not publish canonical changes directly.
5. A separate promotion arbiter branches the skills repository, applies the session diff, resolves or escalates conflicts, and opens or updates a PR.
6. The arbiter may be implemented as a small provider-backed worker, possibly at `xhigh`, a dedicated helper, or an equivalent release tool, but it is a separate publication step rather than a learning stage.
7. Promotion decisions remain observable so later analysis can tie a published skill version back to the originating session and base commit.

## Required Work

### 1. Define overlay-first loading

- Resolve skill reads against the active session overlay when `suggested_skills/` is present.
- Fall back to the approved canonical `skills/` tree only when the overlay does not contain the requested file.
- Keep the overlay session-scoped so parallel runs do not share mutable draft state.

### 2. Separate learning from publication

- Keep the training loop responsible for writing deltas into the session overlay.
- Add or formalize a promotion arbiter that takes the session diff and publishes it into canonical `skills/`.
- Keep the arbiter out of the normal training prompt path so the loop does not need to reason about git branching mechanics on every turn.

### 3. Handle merge conflicts explicitly

- If multiple session diffs touch the same skill file, the arbiter should attempt a semantic merge against the approved base commit.
- If the diffs are compatible but not fast-forwardable, the arbiter may rebase, cherry-pick, or split the change into multiple commits/PRs.
- If the conflict cannot be resolved safely, the arbiter must fail closed and escalate with the conflict evidence intact.

### 4. Update the observability trail

- Record the overlay base commit, session overlay path, promotion outcome, and any conflict or escalation reason.
- Preserve enough metadata to attribute a published `SKILL.md` version back to the originating session and its approved base snapshot.

### 5. Retire legacy helper surfaces

- Treat the existing skill-learning node and git helper surfaces as migration bridges, not as permanent architecture.
- If the arbiter needs logic already present in `controller/agent/nodes/skills.py`, `controller/utils/git.py`, or `worker_light/utils/git.py`, factor that logic into shared helpers and reuse it rather than reimplementing it.
- After the arbiter owns publication and conflict handling, remove or narrow the legacy skill-learning and git-sync paths so they no longer imply a separate required subagent for publication.

## Non-Goals

- Do not make the training loop itself manage canonical branch publication.
- Do not chain unrelated training runs off each other's mutable overlays.
- Do not introduce a dedicated long-lived skill-learner stage.
- Do not auto-merge every draft into canonical `skills/`.
- Do not require the arbiter to be the same runtime component as the learning loop.
- Do not keep legacy skill-learning or git-sync helper paths around after the arbiter has taken over their responsibility.

## Sequencing

The safe order is:

1. Treat `suggested_skills/` as the active overlay for the training session.
2. Teach the training CLI to read that overlay first on later turns.
3. Record the base commit or approved lock metadata used to seed the overlay.
4. Add the promotion arbiter path for publishing approved diffs.
5. Add conflict handling and observability for publication outcomes.
6. Update docs, prompts, and integration coverage to reflect the new contract.

## Acceptance Criteria

1. A resumed skill-training session sees its own prior edits through the session overlay without copying them back into memory by hand.
2. New training runs start from the approved canonical base, not from another run's mutable overlay.
3. A promotion arbiter can publish a session diff into canonical `skills/` and record the resulting commit or PR state.
4. Conflicting skill updates are either merged deliberately or escalated with traceable evidence; they are not silently dropped.
5. The observability trail can attribute a published skill version to the originating session and base commit.
6. The canonical `skills/` tree remains the reviewed source of truth.

## Migration Checklist

### Overlay semantics

- [x] Define the session-local `suggested_skills/` overlay/checkpoint contract in the skill docs.
- [x] Teach the training CLI to resolve the overlay first and the canonical tree second.
- [x] Record the approved base commit or lock metadata used to seed the overlay.

### Promotion semantics

- [x] Add or formalize a promotion arbiter for publishing approved skill diffs.
- [ ] Ensure the arbiter can branch, rebase, cherry-pick, or split changes when needed.
- [x] Fail closed and escalate when a conflict cannot be resolved safely.

### Observability and rollout

- [x] Emit traceable promotion outcome records with source and target skill-repo metadata.
- [ ] Update the prompt and architecture docs to describe the overlay-first contract.
- [x] Add integration coverage for concurrent training runs and promotion conflict handling.

### Legacy cleanup

- [ ] Reuse shared git/skill helper logic in the arbiter where it already exists instead of copying it.
- [ ] Remove or narrow the old skill-learning node and git helper surfaces once the arbiter path is authoritative.
- [ ] Update any runtime config or prompt references that still imply the legacy helper surfaces are required for publication.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `evals/logic/skill_training.py`
- `controller/agent/nodes/skills.py`
- `controller/utils/git.py`
- `worker_light/utils/git.py`
- `scripts/update_skills_lock.py`
- `config/prompts.yaml`
- `config/agents_config.yaml`
- `specs/architecture/agents/agent-skill.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/auxiliary-agent-tools.md`
- `specs/architecture/agents/prompt-management.md`
- `specs/architecture/evals-architecture.md`
- `specs/architecture/observability.md`
- `docs/backend-reference.md`
- integration tests for the skill-training replay path and promotion conflict cases
