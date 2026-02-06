# Tasks: Engineer Agent (002)

## Work Packages

### WP01: Foundation & State

**Goal**: Set up agent project structure, state definitions, and graph skeleton.
**Priority**: High
**Dependencies**: None

- [x] T001: Implement `src/agent/state.py` with `AgentState` definition [P]
- [x] T002: Implement `src/agent/graph.py` skeleton [P]
- [x] T003: Implement `src/agent/prompt_manager.py` [P]
- [x] T004: Create `tests/agent/test_state.py`

### WP02: Architect Node

**Goal**: Implement the Planning (Architect) node.
**Priority**: High
**Dependencies**: WP01

- [x] T005: Implement `src/agent/nodes/architect.py`
- [x] T006: Add logic to read `skills/` directory
- [x] T007: Implement 'create plan' logic (writes `plan.md`)
- [x] T008: Implement 'create todo' logic (writes `todo.md`)
- [x] T009: Test Architect node with mock inputs

### WP03: Engineer Node

**Goal**: Implement the Execution (Engineer) node using Spec 001 tools.
**Priority**: High
**Dependencies**: WP01, Spec 001 WP04

- [x] T010: Implement `src/agent/nodes/engineer.py` with LangGraph ReAct implementation.
- [x] T011: Configure `deepagents` Tools (`ls_tool`, `read_tool`, `write_tool`) using Spec 001 Middleware.
- [x] T012: Configure `exec_tool` for worker integration via **Temporal Client**.
- [x] T013: Implement execution loop with `TodoListMiddleware`.
- [x] T014: Test Engineer node against a running Worker (WP01 WP03).

### WP04: Critic Node & Simulation

**Goal**: Implement the Review (Critic) node.
**Priority**: Medium
**Dependencies**: WP01

- [x] T015: Implement `src/agent/nodes/critic.py`
- [x] T016: Implement logic to parse Simulation outcomes
- [x] T017: Implement logic to parse Workbench outcomes
- [x] T018: Implement decision logic (Approve/Reject)
- [x] T019: Test Critic node

### WP05: Sidecar Learner

**Goal**: Implement the Async Skill Learner.
**Priority**: Medium
**Dependencies**: WP01

- [ ] T020: Implement `src/agent/nodes/sidecar.py` as a background process.
- [ ] T021: Parse Journal for patterns.
- [ ] T022: Implement Skill extraction logic and persistent suggestion storage.
- [ ] T023: Test Sidecar async triggering.

### WP06: Graph Orchestration

**Goal**: Wire all nodes together in `graph.py`.
**Priority**: High
**Dependencies**: WP02, WP03, WP04, WP05

- [ ] T024: Register nodes in `graph.py`
- [ ] T025: Define Conditional Edges
- [ ] T026: Implement Checkpointing
- [ ] T027: Test full Graph flow

### WP07: CLI Entrypoint

**Goal**: Create run scripts and CLI integration.
**Priority**: Low
**Dependencies**: WP06

- [ ] T028: Implement `src/agent/run.py`
- [ ] T029: Implement configuration loading
- [ ] T030: Add Docker/Podman run scripts
- [ ] T031: Final End-to-End smoke test
