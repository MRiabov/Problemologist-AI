---
work_package_id: WP03
title: Graph Architecture Implementation
lane: "for_review"
dependencies: "[]"
base_branch: main
base_commit: 6effdea38856fce3b942735a215df1603531afcf
created_at: '2026-02-01T09:05:41.094433+00:00'
subtasks:
- T009
- T0010
- T0011
- T0012
- T0013
shell_pid: "128761"
---

## Objective

Implement the core `DeepAgents` graph that uses the "Planner-Actor-Critic" cognitive architecture. We will define the nodes and edges of the `StateGraph` and compile it into a runnable app.

## Context

Instead of a monolithic `engine.py`, we are building modular nodes. The state flows from **Planner** (what to do?) -> **Actor** (do it) -> **Critic** (did it work?). The persistence layer (WP02) ensures this process can be paused and resumed.

## Subtasks

### T009: Implement Graph Builder & Conditionals

**Purpose**: Define the skeleton of the graph and the conditional routing logic.
**Steps**:

1. Create `src/agent/graph/graph.py`.
2. Define a `build_graph()` function.
3. Initialize `StateGraph(AgentState)`.
4. Define conditional edges:
   - `should_continue(state)`: Returns "actor", "critic", or "end".
   - Logic: If tool call involves `submit_design` or `preview_design` -> Go to Critic. Else -> Loop back to Actor (or End if max steps).

### T010: Implement Planner Node

**Purpose**: The "Brain" that decides the high-level strategy.
**Steps**:

1. Create `src/agent/graph/nodes/planner.py`.
2. Implement `planner_node(state: AgentState)`.
3. Logic:
   - Checks if `plan` is empty. If so, calls LLM with "Planning Prompt" (referencing `journal.md`).
   - Updates `state["plan"]` with the generated plan.
   - Updates `state["messages"]` with the planning thought process.

### T011: Implement Actor Node

**Purpose**: The "Hand" that executes tools.
**Steps**:

1. Create `src/agent/graph/nodes/actor.py`.
2. Implement `actor_node(state: AgentState)`.
3. Logic:
   - This checks the current step of the plan.
   - Calls the LLM bound with `env_tools` (from WP01).
   - Allows the LLM to call `write_script`, `edit_script`, etc.
   - Appends (AIMessage + ToolMessage) to history.

### T012: Implement Critic Node

**Purpose**: The "Eye" that validates results.
**Steps**:

1. Create `src/agent/graph/nodes/critic.py`.
2. Implement `critic_node(state: AgentState)`.
3. Logic:
   - Triggered after `preview_design` or `submit_design`.
   - Analyzes the Tool Output (Image or Simulation Report).
   - Decisions:
     - **Success**: Update Journal (call `write_journal`), finalize.
     - **Failure**: Update Plan (call `update_plan`), route back to **Actor** or **Planner**.

### T013: Implement Console Streaming

**Purpose**: Visualize the graph execution in real-time.
**Steps**:

1. Create `src/agent/utils/visualize.py`.
2. Use `rich` to subscribe to the graph's event stream (`graph.stream(..., stream_mode="updates")`).
3. Print colorful boxes when entering/exiting nodes.
4. Render streamed tokens if possible, or at least final node outputs.

## Files to Create

- `src/agent/graph/graph.py`
- `src/agent/graph/nodes/planner.py`
- `src/agent/graph/nodes/actor.py`
- `src/agent/graph/nodes/critic.py`
- `src/agent/utils/visualize.py`

## Acceptance Criteria

- [ ] Graph compiles without error.
- [ ] Conditional logic correctly routes between Actor and Critic.
- [ ] Planner generates a plan on first run.
- [ ] Console shows "Entering Planner", "Entering Actor" logs.

## Activity Log

- 2026-02-01T09:25:46Z – unknown – shell_pid=128761 – lane=for_review – Implemented core LangGraph state machine with Planner-Actor-Critic architecture and console visualization. Verified graph compilation.
