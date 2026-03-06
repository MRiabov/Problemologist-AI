# Implementation Plan: Automatic Node Entry Validation
*Path: kitty-specs/012-automatic-node-entry-validation/plan.md*

**Branch**: `main` | **Date**: 2026-03-06 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/012-automatic-node-entry-validation/spec.md`

## Engineering Alignment

- The guard applies to all first-class orchestrated nodes in engineer and benchmark graphs.
- Subagents are included only when they are modeled as graph nodes; tool-invoked helper agents are out of scope for this increment.
- Runtime behavior remains recoverable (deterministic loopback), while integration mode is fail-fast to prevent infinite retries.
- Existing post-node validations and reviewer-handover gates remain mandatory and unchanged in intent.

## Summary

Introduce a pre-entry validation layer for every first-class orchestration node, so state/artifact invariants are checked before node execution. If entry checks fail, regular runtime loops to a deterministic previous node; integration runtime fails fast to `FAILED` with explicit reason telemetry. This design adds defense in depth on top of existing post-node and handover validation.

## Technical Context

**Language/Version**: Python 3.12  
**Primary Dependencies**: FastAPI, LangGraph, Pydantic v2, SQLAlchemy, structlog  
**Storage**: Postgres (episode/traces), worker filesystem artifacts, `events.jsonl` observability stream  
**Testing**: HTTP-only integration tests (`tests/integration/**`, `scripts/run_integration_tests.sh`)  
**Target Platform**: Linux containerized services (`controller`, `worker-light`, `worker-heavy`, Temporal, Postgres, MinIO)  
**Project Type**: Backend distributed orchestration system  
**Performance Goals**: Entry-guard overhead <= 500ms median per guarded transition in local integration runs  
**Constraints**: Fail-closed semantics, no permissive fallback paths, deterministic previous-node routing map, integration fail-fast for repeated entry invalidity  
**Scale/Scope**: All first-class nodes in engineer and benchmark graphs; no new public API surface required

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Microservice-first Architecture**: Pass. Changes stay within controller orchestration and existing worker artifact boundaries.
- **No Reinvention**: Pass. Reuses existing file validators and handover validators; adds a unified orchestration guard instead of duplicate node-local checks.
- **Early Stopping & Fail Fast**: Pass. Explicit fail-fast behavior in integration mode and fail-closed reroute in normal mode.
- **Type Safety & Schemas**: Pass. Guard outputs modeled as typed schema/enums; no raw-string-only contracts.
- **Testing Requirements**: Governed by repository integration-first rule for this feature scope. Validation done through HTTP integration tests and observable traces/events.

Post-design re-check status: **Pass**.

## Project Structure

### Documentation (this feature)

```text
kitty-specs/012-automatic-node-entry-validation/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
├── contracts/
│   └── node-entry-validation.yaml
└── tasks/
```

### Source Code (repository root)

```text
controller/
├── agent/
│   ├── graph.py
│   ├── benchmark/graph.py
│   ├── nodes/
│   └── review_handover.py
├── api/
│   └── tasks.py
└── persistence/
    └── models.py

shared/
├── enums.py
└── observability/
    └── schemas.py

tests/
└── integration/
    └── architecture_p0/
        └── test_node_entry_validation.py  # planned new integration coverage
```

**Structure Decision**: Backend-only orchestration change focused on controller graph routing and integration observability assertions; no frontend or worker-heavy API redesign required.

## Phase 0: Research Plan

### Unknowns to resolve

1. Best insertion point for universal pre-entry checks without duplicating node logic.
2. Deterministic previous-node mapping strategy that avoids ambiguous loop targets.
3. Integration-mode fail-fast semantics that prevent loops but preserve actionable diagnostics.
4. Observable contract for entry-gate failures (trace/event/status fields) using existing endpoints.
5. Scope boundary for subagents not represented as first-class graph nodes.

### Research Outputs

- [research.md](research.md) with finalized architectural decisions and rejected alternatives.

## Phase 1: Design & Contracts Plan

### Data Model Deliverable

- [data-model.md](data-model.md) describing:
  - `NodeEntryValidationResult`
  - `NodeEntryContract`
  - `NodeEntryFailureRecord`
  - routing/disposition enums and episode metadata implications

### Contract Deliverables

- [contracts/node-entry-validation.yaml](contracts/node-entry-validation.yaml)
  - Behavioral API contract over existing endpoints (`POST /api/agent/run`, `GET /api/episodes/{episode_id}`)
  - Trace/event payload schema for node-entry validation failures
  - Status transition expectations for integration and non-integration modes

### Validation Quickstart Deliverable

- [quickstart.md](quickstart.md)
  - Steps to verify reroute behavior
  - Steps to verify integration fail-fast behavior
  - Required trace/event assertions

### Agent Context Update

- Run `spec-kitty agent context update-context --feature 012-automatic-node-entry-validation --agent-type codex --json` after plan artifacts are written.

## Risks & Mitigations

- **Risk**: Guard loops in non-integration mode can hide root cause.  
  **Mitigation**: Persist per-failure reason codes and deterministic previous-node target for each rejection.
- **Risk**: Conflicting guard decisions between graph-level checks and node-local prechecks.  
  **Mitigation**: Graph-level entry guard is source of truth for transition eligibility; node-local checks remain defensive but not routing-authoritative.
- **Risk**: Behavior drift across engineer vs benchmark graphs.  
  **Mitigation**: Shared contract interface with graph-specific mapping tables and common integration assertions.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| None | N/A | N/A |
