---
work_package_id: WP06
title: Graph Orchestration
lane: planned
dependencies: []
subtasks: [T024, T025, T026, T027]
---

## Objective

Wire the nodes (Architect, Engineer, Critic, Sidecar) into a cohesive LangGraph application. Implement conditional routing and checkpointing.

## Context

The individual nodes exist but are not connected. This WP defines the flow logic and state persistence.

## Subtasks

### T024: Register nodes in `graph.py`

Update `src/agent/graph.py`.

- **Action**:
  - Import `architect_node`, `engineer_node`, `critic_node`, `sidecar_node`.
  - `workflow.add_node("architect", architect_node)`
  - `workflow.add_node("engineer", engineer_node)`
  - `workflow.add_node("critic", critic_node)`
  - `workflow.add_node("sidecar", sidecar_node)`

### T025: Define Conditional Edges

Implement the routing logic.

- **Logic**:
  - **Start** -> Architect.
  - **Architect** -> Engineer.
  - **Engineer** -> Critic (after N steps or success).
  - **Critic** ->
    - If Approved: Sidecar -> End.
    - If Rejected (Plan flaw): Architect.
    - If Rejected (Code flaw): Engineer.
  - Define `should_continue` or similar conditional functions.

### T026: Implement Checkpointing

Enable `deepagents` persistence.

- **Action**:
  - Configure `PostgresSaver` (or `MemorySaver` for dev).
  - Pass checkpointer to `workflow.compile(checkpointer=...)`.

### T027: Test full Graph flow

- **File**: `tests/agent/test_graph.py`
- **Test**:
  - Mock all nodes to be pass-throughs.
  - Verify the graph transitions from Start -> Architect -> Engineer -> Critic -> End.
  - Verify state is preserved between steps.

## Definition of Done

- Graph compiles and runs start-to-finish.
- Routing logic correctly handles branching.
