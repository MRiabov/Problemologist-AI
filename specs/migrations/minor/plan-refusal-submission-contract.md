---
title: Plan Refusal Submission Contract
status: investigation
agents_affected:
  - benchmark_coder
  - engineer_coder
added_at: '2026-04-04T09:00:28Z'
---

# Plan Refusal Submission Contract

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration makes plan refusal a first-class, explicit, and validated
completion path for the benchmark coder and engineer coder roles.

After reviewing the backend plumbing, we found that the implementation blast
radius is substantially larger than the original refusal-file idea suggested:
the refusal path touches workspace helpers, worker-heavy validation, reviewer
routing, observability, skill docs, and the skill-loop replay path. It is not
just a `refuse_plan.sh` entrypoint; it is also the backend logic that would use
the refusal manifest as a gate for routing and replay, including future
consumers such as the Codex self-improving skill loop. Because of that, this
migration is now explicitly deferred until the refusal path is necessary
enough to justify the cross-cutting changes.

## Blast Radius

The backend work this migration implies is broader than a single script or a
single manifest write:

1. Workspace-side command surface:
   - `shared/agent_templates/codex/scripts/refuse_plan.sh`
   - any companion refusal helper module
2. Canonical refusal artifact and validation:
   - `plan_refusal.md`
   - `worker_heavy/utils/file_validation.py`
   - `shared/models/schemas.py`
3. Routing and handover plumbing:
   - `controller/api/routes/episodes.py`
   - `controller/agent/node_entry_validation.py`
   - `controller/agent/benchmark_handover_validation.py`
   - `controller/agent/review_handover.py`
   - `controller/agent/nodes/base.py`
   - `controller/agent/nodes/plan_reviewer.py`
   - `controller/agent/nodes/execution_reviewer.py`
   - `controller/agent/handover_constants.py`
4. Observability and retention:
   - `specs/architecture/observability.md`
   - any trace/event schema needed for refusal submission
   - retained bundle handling for replay and training
5. Skill and prompt surfaces:
   - `specs/architecture/agents/agent-harness.md`
   - `specs/architecture/agents/tools.md`
   - `specs/architecture/agents/agent-skill.md`
   - benchmark and engineer coder skills
   - workspace prompt fragments if the refusal gate is advertised there
6. Replay/training consumers:
   - `specs/migrations/minor/self-improving-skill-loop.md`
   - the standalone training/replay CLI or backend core that consumes retained
     bundles

That is why the change is larger than the shell script itself: the manifest
only matters if backend code actually uses it as a gate for routing, replay, and
review decisions.

The target architecture is described in
[agent-harness.md](../../architecture/agents/agent-harness.md),
[tools.md](../../architecture/agents/tools.md),
[handover-contracts.md](../../architecture/agents/handover-contracts.md),
[artifacts-and-filesystem.md](../../architecture/agents/artifacts-and-filesystem.md),
[observability.md](../../architecture/observability.md),
[agent-skill.md](../../architecture/agents/agent-skill.md), and the
role-specific coder skills. The intended end state keeps
`plan_refusal.md` as the canonical proof artifact, but adds an explicit refusal
submission bridge that validates the refusal, records a backend-owned refusal
manifest, and emits traceable observability events.

## Problem Statement

Today the refusal path is only partially explicit:

1. Coder roles are told to write `plan_refusal.md` when the plan is infeasible.
2. The backend validates `plan_refusal.md` content when the file is present.
3. Review routing can confirm or reject the refusal after the fact.
4. `shared.utils.agent.refuse_plan(...)` exists, but it writes `refusal.json`,
   which is not the canonical refusal contract and is easy to miss in prompts,
   skills, and observability.
5. The workspace has an explicit success gate (`submit_plan.sh`), but refusal
   still feels like an ad hoc markdown side effect rather than a deliberate,
   validated submission action.

That shape is workable, but it hides the refusal workflow from the agent
skills and makes the refusal signal weaker than the success signal. A refusal
needs the same kind of deliberate, reviewable completion surface as successful
submission.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `shared/utils/agent/__init__.py` | `refuse_plan(reason)` writes `refusal.json` directly or forwards to the heavy-worker helper. | It does not align with the canonical refusal artifact or produce a backend-owned refusal manifest. |
| `worker_heavy/utils/file_validation.py` | `validate_plan_refusal()` checks YAML frontmatter and requires a non-empty body, but only after the file already exists. | The refusal path needs an explicit submission validator, not just a later review-time parser. |
| `controller/api/routes/episodes.py` | `CONFIRM_PLAN_REFUSAL` validates `plan_refusal.md` and then fails the episode. | The controller can already consume refusal content, but it has no first-class refusal submission record to join against observability and revision metadata. |
| `controller/agent/node_entry_validation.py` | Accepts valid `plan_refusal.md` as a short-circuit path during node-entry validation. | The validation path needs to be paired with an explicit refusal submission helper and manifest so the refusal is intentional and attributable. |
| `controller/agent/benchmark_handover_validation.py` | Treats valid `plan_refusal.md` as an alternate exit path for benchmark motion validation. | This is functionally correct, but still relies on a passive file check rather than an explicit refusal submission contract. |
| `specs/architecture/agents/agent-harness.md` | Names `submit_plan.sh` and `submit_review.sh`, but not a refusal completion gate. | The harness should explain how a coder explicitly refuses a plan, just as it explains how a planner submits one. |
| `specs/architecture/agents/tools.md` | Describes shell-script bridges as the canonical command surface for missing command-like behavior, but refusal is not yet named there. | The refusal path should have the same visible command surface as success paths. |
| `specs/architecture/agents/agent-skill.md` | Describes the refusal concept, but not the exact refusal submission workflow. | Skills need a concrete, teachable command and validation story. |
| `specs/architecture/observability.md` | Mentions plan-refusal events and proof-of-impossibility evidence. | The observability layer should get a concrete refusal submission event/manifest record, not only a later file read. |
| `shared/agent_templates/codex/scripts/` | Planner workspaces get `submit_plan.sh`; coder workspaces do not yet get an equivalent refusal bridge. | The explicit submission bridge should be available in coder workspaces so the refusal workflow is easy to explain and execute. |

## Proposed Target State

1. `plan_refusal.md` remains the canonical refusal proof artifact for both
   benchmark coder and engineer coder roles.
2. Codex coder workspaces get an explicit refusal completion gate,
   `bash scripts/refuse_plan.sh`, parallel to the existing planner success
   gate.
3. The refusal helper validates `plan_refusal.md` before accepting the
   refusal, including frontmatter schema, role-specific reason enums, and
   evidence/body requirements.
4. The refusal helper writes a backend-owned refusal manifest under
   `.manifests/` with session, episode, worker-session, role, revision, and
   refusal-hash metadata. The manifest is the machine-checkable record that
   the refusal was intentionally submitted.
5. The refusal helper emits structured observability events that include the
   refusal reasons, role, revision, and proof artifact references.
6. Backend routing logic uses the refusal manifest as a gate where the refusal
   is consumed, including reviewer entry, replay/training consumers, and any
   future skill-loop logic that needs to branch on refusal instead of on a
   bare file presence check.
7. Reviewer routing continues to rely on `plan_refusal.md` as the canonical
   proof artifact, but it validates the refusal against the refusal manifest
   and observability trail instead of treating the refusal as a bare file
   presence check.
8. `shared.utils.agent.refuse_plan(...)` becomes compatibility-only or is
   redirected to the validated refusal submission path. `refusal.json` is not
   a canonical artifact in the target state.
9. Benchmark coder and engineer coder skills can explain one explicit refusal
   workflow instead of describing a hidden file-side effect.

## Required Work

### 1. Add an explicit refusal submission helper

- Add `shared/agent_templates/codex/scripts/refuse_plan.sh` as the agent-facing
  command surface for coders who must refuse a plan.
- Add a companion Python helper, or equivalent backend utility, that validates
  `plan_refusal.md` and returns a structured refusal result.
- Keep the refusal helper parallel to `submit_plan.py`: the shell script should
  be thin, while the Python/backend layer owns validation, manifest creation,
  and structured output.
- Validate the refusal body against a stricter refusal template than "non-empty
  text" alone, so the refusal contains concrete evidence rather than generic
  prose.
- Fail closed if the refusal file is missing, malformed, stale for the current
  revision, or uses the wrong role-specific reason set.

### 2. Write a refusal manifest and emit observability

- Write a backend-owned refusal manifest for the current refusal submission.
- Include the worker session, episode id, role, benchmark revision, refusal
  reasons, `plan_refusal.md` hash, and any evidence references required for
  deterministic replay.
- Emit explicit refusal submission events into the existing observability
  stream so refusal can be counted, queried, and correlated with later review
  decisions.
- Keep the refusal manifest system-owned, not agent-writable.

### 3. Update reviewer and validation plumbing

- Update the refusal validation path in the controller and worker-heavy
  validators so the refusal submission manifest and `plan_refusal.md` are
  checked together.
- Keep the current behavior that valid refusal routes back through reviewer
  confirmation, but make the refusal signal attributable to the specific
  refusal submission.
- Ensure the refusal checks continue to run against the worker filesystem
  session for the current episode, not a stale cross-session path.
- Use the refusal manifest as a routing gate in the consumers that need to
  branch on refusal, including any replay/training logic that resumes a Codex
  session from retained bundles.

### 4. Make the refusal path explicit in prompts and skills

- Update `agent-harness.md` so coder workspaces clearly advertise the refusal
  command in addition to the successful submission command.
- Update `tools.md` so the refusal bridge is named as the canonical command
  surface for coder-side plan refusal.
- Update `agent-skill.md` and the coder skills so benchmark coder and engineer
  coder agents can explain when to refuse, what evidence to include, and which
  command to run.
- If needed, update the Codex workspace prompt fragments so the refusal gate is
  visible in the run-time instruction set rather than buried in prose.
- Call out that the refusal path is not just a file-write action; it is a
  backend-validated submission flow that downstream code can branch on.

### 5. Retire the legacy helper path

- Deprecate `shared.utils.agent.refuse_plan(...)` as the primary path if the
  new refusal helper supersedes it.
- Remove `refusal.json` from the active contract once the validated refusal
  helper and manifest are in place.
- Keep compatibility only if a transitional path is required by current tests
  or runtime plumbing.
- Do not keep the legacy helper as the only way to signal refusal once the
  manifest-gated path exists.

## Non-Goals

- Do not replace `plan_refusal.md` with a JSON-only refusal record.
- Do not create a new refusal agent or a new reviewer role.
- Do not allow planners to use the refusal submission gate.
- Do not let refusal bypass reviewer validation.
- Do not relax `.manifests/**` ownership rules for agent roles.
- Do not change the success-path `submit_plan.sh` contract as part of this
  migration.
- Do not implement the backend refusal-manifest plumbing until the need is
  strong enough to absorb the wider controller, worker, and observability
  blast radius.

## Sequencing

The safe order is:

1. Add the refusal validation/manifest writer in the backend layer.
2. Add `scripts/refuse_plan.sh` and keep it thin.
3. Update the docs, prompts, and coder skills so the new refusal gate is easy
   to explain and use.
4. Wire reviewer and observability consumers to the refusal manifest.
5. Wire replay/training consumers, including the skill-loop path, to the
   refusal manifest where refusal needs to affect branching.
6. Deprecate `refusal.json` and the legacy helper path after the new contract
   is stable.

Current status: deferred. None of the backend-manifest plumbing should be
implemented until the refusal path is clearly worth the cross-cutting churn.
That includes the manifest-gated routing work for the skill-loop/replay path;
the migration is larger than a shell entrypoint and should not be treated as a
small helper-only change.

## Acceptance Criteria

1. Benchmark coder and engineer coder workspaces have an explicit refusal
   submission command.
2. A valid refusal submission produces a validated `plan_refusal.md`, a
   backend-owned refusal manifest, and an observable refusal event.
3. Invalid or incomplete refusal submissions fail closed before reviewer
   routing.
4. Reviewer confirmation of refusal can be traced back to the specific refusal
   submission and the current workspace session.
5. The coder skills can explain the refusal workflow in one concrete command
   instead of a vague "write the file and stop" instruction.
6. `refusal.json` is no longer a live canonical artifact.

Because the migration is deferred, these criteria are now a future-state
definition rather than an immediate implementation gate.

## Migration Checklist

Use this checklist to track the refusal contract from the explicit command
surface through validation, observability, and reviewer routing. Do not close
the migration until every unchecked item is either completed or explicitly
waived with a written rationale.

### Command surface

- [ ] Add `scripts/refuse_plan.sh` to the Codex coder workspace template.
- [ ] Add the refusal helper backend implementation.
- [ ] Keep the refusal helper thin at the shell layer and strict at the
  validation layer.

### Validation and manifest

- [ ] Validate `plan_refusal.md` before refusal submission is accepted.
- [ ] Write the refusal manifest with session, role, revision, and refusal-hash
  metadata.
- [ ] Keep the refusal manifest backend-owned and inaccessible to agent file
  tools.

### Observability and routing

- [ ] Emit explicit refusal submission events.
- [ ] Make reviewer confirmation consume the refusal manifest plus
  `plan_refusal.md`.
- [ ] Preserve current fail-closed routing for missing or invalid refusal
  artifacts.

### Skills and docs

- [ ] Update `agent-harness.md`.
- [ ] Update `tools.md`.
- [ ] Update `agent-skill.md`.
- [ ] Update benchmark and engineer coder skills so the refusal workflow is
  easy to explain.
- [ ] Update the prompt fragments that advertise coder completion gates.

### Cleanup

- [ ] Deprecate `shared.utils.agent.refuse_plan(...)` if the new helper fully
  replaces it.
- [ ] Remove `refusal.json` from the active contract.
- [ ] Add or refresh integration coverage for the refusal submission path.

## File-Level Change Set

The implementation should touch the smallest set of files that actually
enforce the new contract:

- `shared/agent_templates/codex/scripts/refuse_plan.sh`
- `shared/agent_templates/codex/scripts/refuse_plan.py` or an equivalent
  refusal helper module
- `shared/utils/agent/__init__.py` if the compatibility helper is retained
- `worker_heavy/utils/file_validation.py`
- `controller/api/routes/episodes.py`
- `controller/agent/node_entry_validation.py`
- `controller/agent/benchmark_handover_validation.py`
- `controller/agent/review_handover.py` if the refusal manifest becomes part
  of reviewer entry validation
- `controller/agent/handover_constants.py` or the equivalent refusal-manifest
  constant holder
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/agent-skill.md`
- `specs/architecture/agents/auxiliary-agent-tools.md`
- `specs/architecture/observability.md`
- `skills/benchmark-coder/SKILL.md`
- `skills/engineer-coder/SKILL.md`
- `config/prompts.yaml` if the refusal command needs to be visible in the
  workspace prompt
- integration tests that exercise refusal submission, validation, manifest
  persistence, and reviewer routing

## Open Questions

- Should the refusal manifest be stage-scoped (`benchmark_plan_refusal_manifest.json`
  and `engineering_plan_refusal_manifest.json`) or a single shared file with a
  role field?
- Should the legacy `shared.utils.agent.refuse_plan(...)` helper stay as a
  compatibility shim for one release, or should the new refusal helper replace
  it immediately once the migration lands?
