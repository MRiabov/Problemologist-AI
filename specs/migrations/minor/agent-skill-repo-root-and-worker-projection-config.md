---
title: Agent Skill Repo Root and Worker Projection Config
status: investigation
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
added_at: '2026-04-05T07:36:37Z'
---

# Agent Skill Repo Root and Worker Projection Config

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration moves the canonical skill repository root from `skills/` to `.agents/skills/`.
The repo-local `skills/` mirror and its sync hook are removed; worker runtimes read `.agents/skills/` directly or through the existing `/skills` mount.

The change keeps `suggested_skills/` as the session-local worktree/checkpoint from the existing skill-loop and promotion migrations, but it changes the loading and publication root to `.agents/skills/`.

## Problem Statement

Today the system no longer keeps a repo-local `skills/` projection. Skill loading routes through `.agents/skills/` and the worker-facing `/skills` mount directly.
That model is fragile for two reasons:

1. manual edits in `.agents/skills` are at risk of being overwritten or pruned by sync logic,
2. worker-facing skill availability is implicit instead of being declared by a distro-level policy file.

The architecture now needs three distinct surfaces:

1. the full internal skill repo used by code-editing and controller-side prompt assembly,
2. the worker-facing skill projection, which should be distro-specific and explicitly filtered,
3. the session-local `suggested_skills/` overlay/worktree used by the skill-training and promotion flow.

The new rule is that `.agents/skills/` is the repository, `config/skills_config.yaml` remains metadata for any worker-facing policy checks that still need it, and missing config entries only warn at commit time instead of failing the commit or deleting anything.

## Target State

1. `.agents/skills/` is the git-managed skill repository and the canonical loading entrypoint for all agent skill reads.
2. There is no repo-local `skills/` mirror tree.
3. `suggested_skills/` remains the writable session-local overlay/worktree for skill-training runs.
4. `config/skills_config.yaml` stores per-skill projection metadata, initially via `is_for_worker_agents: true`.
5. Worker runtimes only receive the subset of skills marked for worker use; unlisted skills remain internal-only by default.
6. Missing config entries produce a warning at commit time but do not block the commit and do not mutate the repo.
7. Internal agent prompt assembly and catalog discovery read directly from `.agents/skills/` and the `/skills` mount, not from a copied `skills/` tree.
8. Any runtime worker mount such as `/skills` is a derived projection, not canonical source.

## Relationship to Prior Skill Migrations

This migration does not replace the session-resume or overlay/promotion model already documented in:

1. [self-improving-skill-loop.md](./self-improving-skill-loop.md)
2. [skill-worktree-promotion-arbiter.md](./skill-worktree-promotion-arbiter.md)

Those documents still matter for `suggested_skills/`, session replay, and publication workflow.
The change here is that the canonical publication/loading root is `.agents/skills/`, so the promotion arbiter and any related publication path should target that repo instead of a `skills/` projection.

## Required Work

### 1. Re-root the canonical skill tree

- Update the skill catalog and prompt assembly paths to resolve against `.agents/skills/`.
- Update workspace helpers so CLI-provider workspaces read `.agents/skills/` directly.
- Retire any helper that still treats `skills/` as a canonical repository root or creates a repo-local mirror.

### 2. Add the worker projection policy

- Create `config/skills_config.yaml`.
- Use an extensible per-skill object shape, for example:

```yaml
build123d-cad-drafting-skill:
  is_for_worker_agents: true
```

- Treat this file as deployment policy, not as a second source of truth for skill content.
- Use the policy when syncing worker-facing skill mounts or distro-specific runtime copies.

### 3. Make sync non-destructive to the canonical repo

- Remove delete-on-sync behavior that prunes manual edits from `.agents/skills/`.
- Keep sync logic limited to copying or projecting from the canonical repo into worker-visible destinations.
- Ensure missing config entries are a no-op for the canonical repo and only affect worker projection.

### 4. Add commit-time warnings only

- Add a commit-time check inside the skills repo that warns when a skill exists in `.agents/skills/` but is absent from `config/skills_config.yaml`, if that metadata file is still in use.
- The warning must not fail the commit.
- The warning must not rewrite or delete files.

### 5. Update the surrounding docs and runtime surfaces

- Update architecture docs, prompt text, lock files, and path permissions to refer to `.agents/skills/` as the canonical source.
- Preserve `suggested_skills/` as the active worktree/checkpoint used by the skill-training path.
- Keep `/skills` as a worker-facing mount path where that runtime abstraction still exists.

## Non-Goals

- Do not make `config/skills_config.yaml` a second canonical skill source.
- Do not make missing config entries fail commits.
- Do not collapse `suggested_skills/` into the canonical repo root.
- Do not reintroduce `skills/` as the authoritative source of truth.
- Do not change the existing overlay-first training contract except to retarget publication to `.agents/skills/`.

## Sequencing

The safe order is:

1. Re-root internal catalog and workspace loading to `.agents/skills/`.
2. Introduce `config/skills_config.yaml` and the worker projection loader.
3. Replace destructive mirror behavior with filtered projection behavior.
4. Add non-blocking commit-time warnings for unconfigured skills.
5. Update docs, prompts, permissions, and integration coverage.

## Acceptance Criteria

1. Manual edits inside `.agents/skills/` are not pruned by startup sync or mirror maintenance.
2. Internal agent skill loading uses `.agents/skills/` as the canonical source.
3. Worker runtimes only see skills marked for worker use in `config/skills_config.yaml`.
4. Missing config entries warn at commit time and never fail the commit.
5. No runtime path still assumes `skills/` is canonical.
6. `suggested_skills/` still seeds and promotes independently of the canonical repo root.

## Migration Checklist

### Canonical root

- [ ] Re-root skill catalog loading to `.agents/skills/`.
- [ ] Re-root CLI-provider workspace materialization to `.agents/skills/`.
- [ ] Remove remaining canonical-source references to `skills/`.

### Worker projection

- [ ] Add `config/skills_config.yaml`.
- [ ] Filter worker skill sync using `is_for_worker_agents`.
- [ ] Keep unlisted skills internal-only without failing runtime startup.

### Commit-time warning path

- [ ] Add a warning-only commit check for unconfigured skills.
- [ ] Confirm the warning path exits zero and does not mutate files.

### Documentation and rollout

- [ ] Update architecture docs, prompts, lock files, and permission policies.
- [ ] Add integration coverage for the new canonical root and worker projection behavior.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `shared/skills/catalog.py`
- `controller/agent/prompt_manager.py`
- `evals/logic/codex_workspace.py`
- `evals/logic/skill_training.py`
- `evals/logic/skill_promotion.py`
- `worker_light/app.py`
- `worker_light/utils/git.py`
- `scripts/sync_skill_mirrors.py` (removed)
- `scripts/update_skills_lock.py`
- `config/skills_config.yaml`
- `config/agents_config.yaml`
- `config/prompts.yaml`
- `specs/architecture/agents/agent-skill.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/prompt-management.md`
- `specs/architecture/evals-architecture.md`
- `specs/architecture/observability.md`
- `docs/backend-reference.md`
- `docs/project-overview.md`
- `docs/source-tree-analysis.md`
- `specs/integration-test-list.md`
- integration tests for worker projection and warning-only commit behavior

## Follow-on Refinement

The existing skill-worktree and promotion-arbiter migration remains relevant for `suggested_skills/`.
Its publication target should be retargeted from canonical `skills/` to canonical `.agents/skills/` once this migration is implemented.
