# Feature Specification: Automatic Node Entry Validation

**Feature Branch**: `012-automatic-node-entry-validation`  
**Created**: 2026-03-06  
**Status**: Draft  
**Input**: User description: "Automatic validation on node entry: before every node, validate input; on failure loop back to previous graph node; in integration tests fail fast to avoid infinite loops."

## Intent Summary

This feature adds a mandatory pre-entry validation layer to agent graph nodes so invalid state transitions are blocked before node execution. The target behavior is fail-closed: in regular runtime the graph reroutes to a deterministic previous node for recovery, while integration mode terminates fast as `FAILED` to prevent retry loops.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Block invalid coder entry (Priority: P1)

As a platform maintainer, I need the graph to reject entering coder nodes when required planner artifacts/state are missing, so the system does not execute coding logic against corrupt or incomplete state.

**Why this priority**: This is the direct architecture gap and root cause for state corruption bugs.

**Independent Test**: Start an episode that reaches coder with missing planner prerequisites and verify coder logic is not executed for that turn.

**Acceptance Scenarios**:

1. **Given** an engineer episode where coder entry is attempted without valid plan inputs, **When** node-entry validation runs, **Then** entry is rejected before coder execution and the run is rerouted.
2. **Given** a benchmark episode resumed in `PLANNED` state with missing coder prerequisites, **When** benchmark coder entry is evaluated, **Then** entry is rejected with explicit validation errors.

---

### User Story 2 - Deterministic recovery vs integration fail-fast (Priority: P1)

As a test and release owner, I need production and integration behavior to diverge safely, so runtime can recover while integration tests fail quickly and deterministically.

**Why this priority**: Infinite loops during integration currently hide root cause and waste compute.

**Independent Test**: Run identical invalid-entry scenario in normal mode and integration mode and confirm reroute vs immediate failure behavior.

**Acceptance Scenarios**:

1. **Given** non-integration runtime, **When** entry validation fails, **Then** orchestration routes to a deterministic previous node and records recovery intent.
2. **Given** integration runtime (`IS_INTEGRATION_TEST=true`), **When** the same entry validation fails, **Then** the episode transitions to `FAILED` without retry loopback.

---

### User Story 3 - Debuggable guardrail telemetry (Priority: P2)

As a developer debugging failed episodes, I need structured traces/events for node-entry failures, so I can quickly identify node, reason, and routing decision.

**Why this priority**: Without explicit telemetry, entry-gate failures look like generic node failures.

**Independent Test**: Trigger an entry-gate failure and assert persisted trace/event fields include node, error list, and disposition (`reroute` or `fail_fast`).

**Acceptance Scenarios**:

1. **Given** a node-entry validation failure, **When** the failure is persisted, **Then** traces/events include failing node id, reason codes, and disposition.
2. **Given** multiple failures in one episode, **When** reviewing persisted telemetry, **Then** each failure is independently queryable and chronologically ordered.

---

### Edge Cases

- Start-node validation failure where no previous node exists in the workflow path.
- Resume/continue episodes where persisted status suggests a later phase but required artifacts are stale or deleted.
- Reviewer entry attempted with stale `.manifests/review_manifest.json` that no longer matches script hash.
- Repeated invalid-entry conditions that could loop indefinitely in non-integration mode.
- Mixed mechanical/electronics TODOs where coder/electronics engineer entry gates select different prerequisite checks.

## Requirements *(mandatory)*

### Functional Requirements

| ID | Requirement | Status |
|---|---|---|
| FR-001 | System MUST run a node-entry validation function before every executable node in engineer and benchmark graphs. | Proposed |
| FR-002 | Entry validation MUST evaluate both in-memory graph state and required workspace artifacts for the target node. | Proposed |
| FR-003 | If validation fails in non-integration mode, orchestration MUST reroute to a deterministic previous graph node instead of entering the target node. | Proposed |
| FR-004 | If validation fails in integration mode, orchestration MUST fail fast by transitioning episode/session status to `FAILED` and must not reroute. | Proposed |
| FR-005 | System MUST define and use an explicit previous-node map per graph path; previous-node resolution MUST NOT be inferred from transient runtime heuristics. | Proposed |
| FR-006 | Engineer coder entry validation MUST require valid planner handoff prerequisites (including plan/todo state and required files) before coder execution. | Proposed |
| FR-007 | Benchmark coder entry validation MUST require valid benchmark planner handoff prerequisites before benchmark coder execution. | Proposed |
| FR-008 | Execution reviewer and benchmark reviewer entry validation MUST enforce latest-revision reviewer handoff validity using existing handoff artifact invariants. | Proposed |
| FR-009 | On every entry validation failure, system MUST persist structured trace/event records with failing node, error reasons, and disposition (`reroute` or `fail_fast`). | Proposed |
| FR-010 | Existing post-node output validation gates MUST remain active and unchanged in intent; entry validation is an additional layer, not a replacement. | Proposed |
| FR-011 | Single-node planner graphs used by direct planner runs MUST apply entry validation and fail closed on failure (no loopback path). | Proposed |
| FR-012 | Integration tests MUST cover both reroute and fail-fast contracts via HTTP-only black-box flows. | Proposed |

### Non-Functional Requirements

| ID | Requirement | Status |
|---|---|---|
| NFR-001 | Integration fail-fast MUST complete transition to `FAILED` within one graph turn after guard failure is detected. | Proposed |
| NFR-002 | Entry validation checks MUST be deterministic: identical state + files produce identical validation result and disposition. | Proposed |
| NFR-003 | 100% of entry-validation failures MUST produce at least one persisted trace/event record observable through API or `events.jsonl`. | Proposed |
| NFR-004 | Entry validation MUST not add more than 500ms median latency per guarded transition under local integration-test conditions. | Proposed |
| NFR-005 | New guardrail logic MUST preserve pass status of existing mapped P0 planner/reviewer gate tests unless test expectations are explicitly updated. | Proposed |

### Constraints

| ID | Constraint | Status |
|---|---|---|
| C-001 | Integration coverage for this feature MUST be implemented as real integration tests only; no unit-test substitution. | Enforced |
| C-002 | Tests MUST drive behavior through HTTP endpoints and assert against observable system boundaries (status, traces, events, artifacts). | Enforced |
| C-003 | Validation contracts MUST use typed schema models/enums already used by orchestration codepaths. | Enforced |
| C-004 | Reviewer gate behavior MUST remain fail-closed with latest-revision manifest semantics; no permissive fallback paths. | Enforced |
| C-005 | `.manifests/**` policy constraints remain unchanged; tests must validate behavior through API/state transitions, not model-side file reads. | Enforced |

### Key Entities *(include if feature involves data)*

- **NodeEntryValidationResult**: Structured decision for a target node entry (`ok`, `errors`, `disposition`, `reroute_target`).
- **NodeEntryContract**: Node-specific preconditions over state fields and required artifacts.
- **NodeEntryFailureRecord**: Persisted observability payload for rejected node entries.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: In a forced "missing plan before coder" scenario, coder node execution count is `0` for the rejected turn and reroute/fail-fast behavior matches runtime mode.
- **SC-002**: In integration mode, invalid node entry transitions to `FAILED` without repeated loopback cycles.
- **SC-003**: Every entry-validation rejection is visible in episode traces/events with node id and reason details.
- **SC-004**: Existing planner/reviewer gate coverage remains green after introducing node-entry guards, with no silent weakening of failure semantics.

## Assumptions

- The feature applies to both orchestration graphs (`controller/agent/graph.py` and `controller/agent/benchmark/graph.py`).
- Existing reviewer handoff validation utilities are reused instead of duplicating reviewer-gate logic.
- New integration tests are added to architecture P0 coverage and mapped to a new `INT-xxx` identifier in the integration spec.
