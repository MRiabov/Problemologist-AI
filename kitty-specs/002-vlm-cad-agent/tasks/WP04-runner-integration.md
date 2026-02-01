---
work_package_id: WP04
title: Runner & Integration
lane: planned
dependencies: ["WP03"]
subtasks:
- T014
- T015
- T016
---

## Objective

Create the entry point for the agent (`runner.py`) that initializes the LangGraph application and executes it asynchronously. Validate the full loop with end-to-end tests.

## Context

The agent is now a compiled `StateGraph`. To run it, we need an asynchronous runner that handles the event stream and connects the `visualize` utils (from WP03) to the graph's output.

## Subtasks

### T014: Implement Environment Adapter for LangChain

**Purpose**: Bridge the Agent's tool calls to the Environment.
**Steps**:

1. Create `src/agent/tools/env_adapter.py`.
2. Ensure the tools defined in WP01 (`src/agent/tools/env.py`) correctly map to the underlying `001-agentic-cad-environment` logic.
   - *Assumption*: If Spec 001 provides a python API, import it. If it's a server, use `httpx`.
   - Wrap these calls in `async` functions to avoid blocking the graph.

### T015: Implement Async Runner CLI

**Purpose**: The main executable script.
**Steps**:

1. Create `src/agent/runner.py`.
2. Use `asyncclick` or standard `argparse` + `asyncio.run()`.
3. Logic:
   - Load Config (API Keys).
   - Initialize `Checkpointer` (WP02).
   - Build Graph (`app = build_graph().compile(checkpointer=...)`).
   - Run loop:

     ```python
     async for event in app.astream(initial_state, config):
         visualize_event(event)
     ```

### T016: Verify End-to-End Graph Execution

**Purpose**: Smoke test the complete graph.
**Steps**:

1. Create `tests/e2e/test_graph_run.py`.
2. Use `langchain-core.tracers.context` or `langsmith` to capture run traces.
3. Mock the LLM to return a predefined plan and tool call.
4. Assert that the graph transitions: `start` -> `planner` -> `actor` -> `critic` -> `end` (or similar happy path).

## Files to Create

- `src/agent/tools/env_adapter.py`
- `src/agent/runner.py`
- `tests/e2e/test_graph_run.py`

## Acceptance Criteria

- [ ] `python src/agent/runner.py` executes the graph asynchronously.
- [ ] Console output shows the graph events streaming.
- [ ] Graph successfully calls environment tools via adapters.
