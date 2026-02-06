---
work_package_id: WP06
title: Graph Orchestration
lane: "doing"
dependencies: []
base_branch: 002-vlm-cad-agent-WP05
base_commit: 71b0688ff1eab316e8ffebdda302044a2c68f2ee
created_at: '2026-02-06T14:42:04.319118+00:00'
subtasks: [T024, T025, T026, T027]
shell_pid: "47212"
agent: "antigravity"
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

## Activity Log

- 2026-02-06T14:42:04Z – Gemini – shell_pid=659900 – lane=doing – Assigned agent via workflow command
- 2026-02-06T15:02:28Z – Gemini – shell_pid=659900 – lane=for_review – Ready for review: Full graph orchestration implemented with routing and checkpointing.
- 2026-02-06T17:46:40Z – antigravity – shell_pid=47212 – lane=doing – Started review via workflow command
