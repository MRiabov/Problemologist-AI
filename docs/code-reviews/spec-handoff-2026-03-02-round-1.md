# Spec Handoff - March 02, 2026 - Round 1

## Overview

This document captures the recent architecture-spec decisions and turns them into an implementation prompt for future agents.
Primary source: `specs/desired_architecture.md`.

---

## 🚨 Critical Implementation Prompt

### 1. Enforce per-agent filesystem policy from `config/agents_config.yaml`

**Spec refs**: `specs/desired_architecture.md` (`Initial files for each agent and read-write permissions`, `agents_config.yaml (path permissions policy)`, `Agent-native tools`).

**Problem to solve**:
Tool availability is broad (`read/write/edit/execute`), so role-specific boundaries must be enforced at path-level by policy.

**Required implementation**:

1. Introduce/complete runtime loading of `config/agents_config.yaml`.
2. Enforce gitignore-style path matching (`*`, `**`) for read/write/edit/upload/download.
3. Apply precedence exactly as spec:
   - `deny` > `allow`
   - unmatched path => deny
   - agent rules override defaults.
4. Return deterministic permission errors (not silent fallback).
5. Ensure reviewers can only write under `reviews/review-round-*/review.md`.

**Likely touchpoints**:

- `shared/workers/filesystem/*`
- `controller/middleware/remote_fs.py`
- Any tool adapter paths in agent runtime wrappers.

**Acceptance criteria**:

- Reviewer `write/edit` to non-review file is blocked.
- Engineering/Benchmark CAD implementers cannot edit `objectives.yaml` / `assembly_definition.yaml` after planner handoff lock.
- Policy-driven allow/deny behavior is covered by tests.

---

### 2. Implement structured plan refusal artifact workflow (`plan_refusal.md`)

**Spec refs**: `Plan refusal artifact (plan_refusal.md)`, handover section, reviewer decision frontmatter.

**Problem to solve**:
Refusal logic must be structured, role-specific, and machine-validated.

**Required implementation**:

1. Add parser/validator for `plan_refusal.md` YAML frontmatter.
2. Validate role-specific `reasons` enums:
   - `engineering_mechanical_coder`
   - `engineering_electrical_coder`
   - `benchmark_cad_coder`
3. Support multi-reason refusals (`reasons: [...]`).
4. Wire reviewer decisions:
   - `confirm_plan_refusal`
   - `reject_plan_refusal`
5. Require refusal evidence in markdown body (non-empty, structured enough for reviewer).

**Likely touchpoints**:

- Reviewer nodes (`controller/agent/nodes/*reviewer*.py`)
- Planner/coder handoff orchestration
- Markdown validation utilities

**Acceptance criteria**:

- Refusal without valid `plan_refusal.md` is rejected.
- Invalid reason for role fails validation.
- Reviewer can deterministically confirm/reject refusal based on parsed artifact.

---

### 3. Bind COTS reproducibility to persisted outputs

**Spec refs**: `COTS catalog database (spec 006)` and observability sections.

**Problem to solve**:
COTS metadata exists, but reproducibility requires strict persistence of catalog snapshot + query + selection snapshots with handoff outputs.

**Required implementation**:

1. Add `catalog_snapshot_id` in catalog metadata (build-time generated).
2. Persist for each COTS call/handoff:
   - `catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`
   - normalized query snapshot
   - ordered candidate list + final selected `part_id`s.
3. Mark handoff invalid if selected parts exist without reproducibility metadata.

**Likely touchpoints**:

- `shared/cots/indexer.py`
- `shared/cots/runtime.py`
- Event schemas and persistence (`shared/observability/schemas.py`, controller tracing/ingest)
- Handoff artifact generation paths.

**Acceptance criteria**:

- COTS events and downstream artifacts include snapshot metadata.
- Replay query reconstruction is possible from stored records.
- Integration coverage aligns with `INT-064` expectations.

---

### 4. Fix ID model bug: `session_id` vs `episode_id` conflation

**Spec refs**: `ID model and linkage` section (explicitly marked as bug).

**Problem to solve**:
`session_id` and `episode_id` are conflated in parts of runtime/observability; this breaks clean joins for multi-episode user sessions.

**Required implementation**:

1. Keep `episode_id` as workflow-run identity.
2. Introduce/propagate distinct `user_session_id` for UI conversation scope.
3. Persist both where relevant (traces/events/assets).
4. Add IDs for child artifacts where missing:
   - `simulation_run_id`
   - `cots_query_id`
   - `review_id`.

**Likely touchpoints**:

- `controller/agent/*state*.py`
- `controller/observability/*`
- `shared/observability/schemas.py`
- Worker request/response schemas if ID propagation crosses boundary.

**Acceptance criteria**:

- Observability records can be joined by `user_session_id -> episode_id -> child IDs`.
- Legacy behavior remains backward-compatible or has migration notes.

---

## Suggested Execution Order

1. Implement ID model separation (`user_session_id` vs `episode_id`) first.
2. Enforce `agents_config.yaml` policy gates.
3. Implement `plan_refusal.md` validation and reviewer routing.
4. Bind COTS reproducibility metadata to handoffs/events.
5. Add/adjust integration tests and observability assertions.

## Notes for Future Agents

- Treat this as architecture-contract work, not prompt tuning.
- Prefer failing fast with explicit validation errors over permissive fallbacks.
- Keep worker/controller boundaries strict; validate at HTTP/system boundaries.
