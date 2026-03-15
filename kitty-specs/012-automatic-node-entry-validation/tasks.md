# Work Packages: Automatic Node Entry Validation

**Inputs**: Design documents from `/kitty-specs/012-automatic-node-entry-validation/`  
**Prerequisites**: `plan.md`, `spec.md`, `research.md`, `data-model.md`, `contracts/node-entry-validation.yaml`, `quickstart.md`

**Tests**: Integration tests are required by feature requirements (`FR-012`) and repository policy.

**Organization**: Fine-grained subtasks (`Txxx`) roll up into work packages (`WPxx`). Each work package is independently deliverable and reviewable.

## Setup & Foundations

### WP01 - Entry Guard Foundation & Contracts (Priority: P1)

**Goal**: Establish the shared node-entry validation model, routing policy primitives, and contract hooks used by both graphs.  
**Independent Test**: Guard evaluation can return deterministic `allow/reroute_previous/fail_fast` decisions from typed inputs.  
**Prompt**: `/tasks/WP01-entry-guard-foundation-and-contracts.md`

**Dependencies**: None
**Requirement Refs**: FR-001, FR-002, FR-005, FR-013, NFR-002, C-003

#### Included Subtasks

- [x] T001 Create shared node-entry validation module and typed result/error models under `controller/agent/`.
- [x] T002 Add shared enums/reason-code constants for entry failure disposition and source.
- [x] T003 Implement deterministic previous-node mapping tables for engineer and benchmark graphs.
- [x] T004 Add integration-mode policy resolver (`IS_INTEGRATION_TEST`) used by entry guards.
- [x] T005 Add reusable validation contract interface for node-specific checks (state/artifact/custom hook).

**Estimated Prompt Size**: ~320 lines

---

### WP02 - Engineer Graph Guard Integration (Priority: P1)

**Goal**: Apply pre-entry validation to all first-class nodes in engineer orchestration and planner single-node graph runs.  
**Independent Test**: Engineer graph blocks invalid node entry before node execution and reroutes/fails-fast per mode.  
**Prompt**: `/tasks/WP02-engineer-graph-guard-integration.md`

**Dependencies**: WP01
**Requirement Refs**: FR-001, FR-003, FR-004, FR-005, FR-006, FR-010, FR-011, NFR-001, NFR-002, C-003

#### Included Subtasks

- [x] T006 Integrate guard evaluation into `controller/agent/graph.py` routing before each first-class node transition.
- [x] T007 Define engineer-node contracts for planner, electronics planner, coder, electronics engineer, and reviewer nodes.
- [x] T008 Ensure rejected entries skip target node execution and route to deterministic previous node in non-integration mode.
- [x] T009 Ensure integration-mode entry rejection transitions to failed state without loopback retry.
- [x] T010 Apply same guard semantics to `engineer_planner_graph` and `electronics_planner_graph` (single-node fail-closed behavior).
- [x] T011 Persist structured entry-rejection context into state feedback/journal fields used by downstream status persistence.

**Estimated Prompt Size**: ~430 lines

---

### WP03 - Benchmark Graph Guard Integration (Priority: P1)

**Goal**: Apply pre-entry validation to benchmark planner/coder/reviewer flow and continuation paths.  
**Independent Test**: Benchmark graph rejects invalid node entry deterministically across initial run and continue flow.  
**Prompt**: `/tasks/WP03-benchmark-graph-guard-integration.md`

**Dependencies**: WP01
**Requirement Refs**: FR-001, FR-003, FR-004, FR-005, FR-007, FR-010, NFR-001, NFR-002, C-003

#### Included Subtasks

- [x] T012 Add entry-guard routing for benchmark planner, coder, reviewer, skill, and journalling nodes in `controller/agent/benchmark/graph.py`.
- [x] T013 Define benchmark-node contracts including planner artifact/state prerequisites and reviewer handover prerequisites.
- [x] T014 Enforce deterministic reroute target selection for non-integration mode benchmark failures.
- [x] T015 Enforce integration fail-fast for benchmark entry rejection in streaming execution path.
- [x] T016 Apply guard checks to start/continue paths (`run_generation_session` and `continue_generation_session`) so invalid resumed states are rejected.
- [x] T017 Ensure benchmark state persistence captures entry-validation failure details and reason codes.

**Estimated Prompt Size**: ~450 lines

---

## Cross-Cutting Validation Gates

### WP04 - Reviewer & Handover Entry Contracts (Priority: P1)

**Goal**: Harmonize reviewer entry checks so latest-revision handover invariants are enforced before entering reviewer logic in both graphs.  
**Independent Test**: Stale/missing handover artifacts block reviewer entry with structured failure reason before reviewer node executes.  
**Prompt**: `/tasks/WP04-reviewer-and-handover-entry-contracts.md`

**Dependencies**: WP02, WP03
**Requirement Refs**: FR-008, FR-010, FR-013, C-004, C-005

#### Included Subtasks

- [x] T018 Refactor/reuse `validate_reviewer_handover` as contract hook for execution reviewer and benchmark reviewer entry.
- [x] T019 Add explicit reviewer-entry contract checks for engineer and benchmark reviewer nodes (including manifest/script hash alignment).
- [x] T020 Ensure reviewer entry failures are surfaced as node-entry failures, not generic runtime exceptions.
- [x] T021 Add contract boundary for first-class subagent nodes only; tool-invoked helper agents remain explicitly out of scope.
- [x] T022 Add guardrail comments/docstrings in orchestration files clarifying entry-validation scope and exception boundary.

**Estimated Prompt Size**: ~300 lines

---

### WP05 - Observability & Failure Surfacing (Priority: P2)

**Goal**: Emit consistent trace/event/metadata records for all entry-validation rejections and route decisions.  
**Independent Test**: Every entry rejection appears in episode traces/events with node, disposition, and reason code fields.  
**Prompt**: `/tasks/WP05-observability-and-failure-surfacing.md`

**Dependencies**: WP02, WP03, WP04
**Requirement Refs**: FR-009, NFR-003, NFR-005, C-002, C-004

#### Included Subtasks

- [x] T023 Add dedicated observability event schema for node-entry validation failure (or map to existing strict event family if required).
- [x] T024 Emit trace/error records on entry rejection with normalized metadata payload (`node`, `disposition`, `reason_code`, `reroute_target`, `errors`).
- [x] T025 Ensure episode `metadata_vars.validation_logs` receives stable reason-code strings for entry failures.
- [x] T026 Ensure status broadcasts include fail-fast transition metadata in integration mode.
- [x] T027 Add structured logging keys for operational debugging without leaking sensitive payloads.
- [x] T028 Verify event/trace emission order remains deterministic and parseable by existing integration assertions.

**Estimated Prompt Size**: ~360 lines

---

## Integration Contract Coverage

### WP06 - Integration Tests & Scenario Wiring (Priority: P1)

**Goal**: Add HTTP-only integration coverage for reroute and fail-fast entry validation contracts and map coverage to integration spec IDs.  
**Independent Test**: New integration tests pass and validate both behavioral modes using API-visible state/traces/events only.  
**Prompt**: `/tasks/WP06-integration-tests-and-scenario-wiring.md`

**Dependencies**: WP04, WP05
**Requirement Refs**: FR-012, NFR-001, NFR-003, C-001, C-002

#### Included Subtasks

- [x] T029 Add a new `INT-xxx` contract row in `specs/integration-tests.md` for automatic node-entry validation.
- [x] T030 Create `tests/integration/architecture_p0/test_node_entry_validation.py` with entry rejection coverage for multiple node types.
- [x] T031 Add integration scenario(s) to `tests/integration/mock_responses/` that intentionally trigger invalid entry preconditions.
- [x] T032 Add assertions that target node execution is skipped on rejected entry turns.
- [x] T033 Add assertions for integration fail-fast behavior (`FAILED` transition without retry loop).
- [x] T034 Add assertions for non-integration reroute behavior where supported by test harness mode.
- [x] T035 Add assertions for trace/event metadata contract (`node`, `disposition`, `reason_code`, `errors`).

**Estimated Prompt Size**: ~500 lines

---

## Hardening & Regression Safety

### WP07 - Compatibility, Regression, and Documentation Sync (Priority: P2)

**Goal**: Preserve existing planner/reviewer gate behavior while integrating entry guards, and finalize documentation parity.  
**Independent Test**: Existing critical integration suites remain green and docs align with implemented guard behavior.  
**Prompt**: `/tasks/WP07-compatibility-regression-and-doc-sync.md`

**Dependencies**: WP06
**Requirement Refs**: FR-010, NFR-005, C-004, C-005

#### Included Subtasks

- [x] T036 Run targeted integration suites for planner gates, reviewer handover, and new entry-validation tests.
- [x] T037 Resolve regressions caused by new guard routing while preserving fail-closed semantics.
- [x] T038 Update feature `quickstart.md` and plan notes if implementation details diverge from planned assumptions.
- [x] T039 Validate that `.manifests/**` policy and reviewer write-scope behavior remain unchanged.
- [x] T040 Produce final implementation notes for reviewers describing expected node-entry failure signatures.

**Estimated Prompt Size**: ~260 lines

---

## Dependency & Execution Summary

- **Execution order**: WP01 -> (WP02 + WP03 in parallel) -> WP04 -> WP05 -> WP06 -> WP07.
- **Parallelization highlights**:
  - WP02 and WP03 can run concurrently after WP01.
  - Within WP06, scenario wiring and assertion authoring can be split by test files once baseline scaffold is created.
- **MVP scope recommendation**: WP01 + WP02 + WP03 + WP04 + WP05 (core runtime behavior and observability). WP06/WP07 complete release-quality contract validation.

## Subtask Index (Reference)

| Subtask ID | Summary | Work Package | Priority | Parallel? |
|------------|---------|--------------|----------|-----------|
| T001 | Shared validation module/models | WP01 | P1 | No |
| T002 | Disposition/source enums and reason constants | WP01 | P1 | Yes |
| T003 | Previous-node mapping tables | WP01 | P1 | Yes |
| T004 | Integration policy resolver | WP01 | P1 | Yes |
| T005 | Validation contract interface | WP01 | P1 | No |
| T006 | Engineer graph pre-entry guard integration | WP02 | P1 | No |
| T007 | Engineer node contracts | WP02 | P1 | Yes |
| T008 | Non-integration reroute in engineer graph | WP02 | P1 | No |
| T009 | Integration fail-fast in engineer graph | WP02 | P1 | No |
| T010 | Single-node planner fail-closed guard | WP02 | P1 | Yes |
| T011 | Engineer failure feedback/journal surfacing | WP02 | P1 | Yes |
| T012 | Benchmark graph pre-entry guard integration | WP03 | P1 | No |
| T013 | Benchmark node contracts | WP03 | P1 | Yes |
| T014 | Non-integration reroute in benchmark graph | WP03 | P1 | No |
| T015 | Integration fail-fast in benchmark streaming path | WP03 | P1 | No |
| T016 | Start/continue path guard coverage | WP03 | P1 | Yes |
| T017 | Benchmark metadata/log propagation | WP03 | P1 | Yes |
| T018 | Reuse reviewer handover hook in entry contracts | WP04 | P1 | No |
| T019 | Reviewer-entry contracts (engineer/benchmark) | WP04 | P1 | Yes |
| T020 | Normalize reviewer entry failure classification | WP04 | P1 | Yes |
| T021 | Subagent scope boundary enforcement | WP04 | P1 | Yes |
| T022 | Scope clarifying comments/docstrings | WP04 | P1 | Yes |
| T023 | Entry-validation observability event schema | WP05 | P2 | No |
| T024 | Trace/error emission payload | WP05 | P2 | No |
| T025 | Episode validation_logs reason-code propagation | WP05 | P2 | Yes |
| T026 | Status broadcast metadata for fail-fast | WP05 | P2 | Yes |
| T027 | Structured logging fields | WP05 | P2 | Yes |
| T028 | Deterministic event/trace emission order | WP05 | P2 | No |
| T029 | Add integration spec ID row | WP06 | P1 | Yes |
| T030 | New integration test file scaffold | WP06 | P1 | No |
| T031 | Mock scenario wiring for invalid entry | WP06 | P1 | Yes |
| T032 | Assert target-node skip on rejection | WP06 | P1 | Yes |
| T033 | Assert integration fail-fast status behavior | WP06 | P1 | Yes |
| T034 | Assert non-integration reroute behavior | WP06 | P1 | Yes |
| T035 | Assert trace/event payload contract | WP06 | P1 | Yes |
| T036 | Run targeted integration suites | WP07 | P2 | No |
| T037 | Resolve guard-related regressions | WP07 | P2 | No |
| T038 | Sync quickstart/plan notes with implementation | WP07 | P2 | Yes |
| T039 | Validate manifests/reviewer policy unchanged | WP07 | P2 | Yes |
| T040 | Final reviewer-facing implementation notes | WP07 | P2 | Yes |

<!-- status-model:start -->
## Canonical Status (Generated)
- WP01: done
- WP02: done
- WP03: done
- WP04: done
- WP05: done
- WP06: done
- WP07: done
<!-- status-model:end -->
